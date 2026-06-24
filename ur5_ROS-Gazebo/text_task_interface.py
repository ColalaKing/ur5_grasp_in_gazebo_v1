#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import json
import re
import sys
import threading
import time

import rospy
from std_msgs.msg import String


try:
    text_type = unicode
except NameError:
    text_type = str


CHINESE_DIGITS = {
    u"零": 0,
    u"〇": 0,
    u"一": 1,
    u"二": 2,
    u"两": 2,
    u"三": 3,
    u"四": 4,
    u"五": 5,
    u"六": 6,
    u"七": 7,
    u"八": 8,
    u"九": 9,
}

FULLWIDTH_DIGITS = {
    u"０": u"0",
    u"１": u"1",
    u"２": u"2",
    u"３": u"3",
    u"４": u"4",
    u"５": u"5",
    u"６": u"6",
    u"７": u"7",
    u"８": u"8",
    u"９": u"9",
}

DIGIT_TOKEN = u"0-9０-９零〇一二两三四五六七八九"


def to_unicode(value):
    if isinstance(value, text_type):
        return value
    try:
        return value.decode("utf-8")
    except Exception:
        return text_type(value)


def normalize_text(value):
    text = to_unicode(value).strip()
    for fullwidth, ascii_digit in FULLWIDTH_DIGITS.items():
        text = text.replace(fullwidth, ascii_digit)
    text = re.sub(u"[\t\r\n]+", u" ", text)
    text = re.sub(u"\\s+", u" ", text)
    return text


def parse_digit_token(token):
    token = normalize_text(token)
    if re.match(u"^[0-9]$", token):
        return int(token)
    if token in CHINESE_DIGITS:
        return CHINESE_DIGITS[token]
    raise ValueError("unsupported number token: %s" % token.encode("utf-8"))


def parse_task_text(text):
    """Parse commands such as '抓取 3 号方块放到 5 号格子'.

    Supported examples:
    - 抓取 3 号方块放到 5 号格子
    - 把三号方块放到五号格
    - 3放5
    - pick 3 to grid 5
    - 3 -> 5
    """
    normalized = normalize_text(text)
    patterns = [
        u"(?:抓取|抓|拿|拾取|把)?\\s*([%s])\\s*(?:号)?\\s*(?:数字)?\\s*(?:方块|块)?\\s*(?:放到|放入|放进|放置到|放在|放)\\s*([%s])\\s*(?:号)?\\s*(?:格子|格|目标格)?"
        % (DIGIT_TOKEN, DIGIT_TOKEN),
        u"(?:把|将)\\s*([%s])\\s*(?:号)?\\s*(?:数字)?\\s*(?:方块|块)\\s*(?:放到|放入|放进|放置到|放在|放)\\s*([%s])\\s*(?:号)?\\s*(?:格子|格|目标格)?"
        % (DIGIT_TOKEN, DIGIT_TOKEN),
        u"pick\\s*([0-9])\\s*(?:to|->)\\s*(?:grid\\s*)?([0-9])",
        u"([0-9])\\s*[-=]*>\\s*([0-9])",
        u"^\\s*([%s])\\s+([%s])\\s*$" % (DIGIT_TOKEN, DIGIT_TOKEN),
    ]
    lower = normalized.lower()
    for pattern in patterns:
        match = re.search(pattern, lower, re.IGNORECASE)
        if match:
            pick_digit = parse_digit_token(match.group(1))
            place_grid = parse_digit_token(match.group(2))
            if pick_digit < 0 or pick_digit > 9:
                raise ValueError("pick_digit %d out of range [0..9]" % pick_digit)
            if place_grid < 1 or place_grid > 6:
                raise ValueError("place_grid %d out of range [1..6]" % place_grid)
            return {
                "pick_digit": pick_digit,
                "place_grid": place_grid,
                "raw_text": normalized,
            }
    raise ValueError("cannot parse task text: %s" % normalized.encode("utf-8"))


class TextTaskInterface(object):
    def __init__(self, argv):
        rospy.init_node("text_task_interface", anonymous=False)

        self.input_topic = rospy.get_param("~input_topic", "/task/text_command")
        self.task_command_topic = rospy.get_param(
            "~task_command_topic", "/grasp/task_command"
        )
        self.status_topic = rospy.get_param("~status_topic", "/task/text_status")
        self.enable_stdin = self.as_bool(rospy.get_param("~enable_stdin", True))
        self.wait_for_executor = self.as_bool(
            rospy.get_param("~wait_for_executor", True)
        )
        self.wait_timeout = float(rospy.get_param("~wait_timeout", 5.0))

        self.task_pub = rospy.Publisher(
            self.task_command_topic, String, queue_size=1
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.input_topic, String, self.topic_callback, queue_size=1)

        self.argv_text = u" ".join([to_unicode(item) for item in argv])
        rospy.loginfo(
            "text_task_interface ready: stdin=%s input_topic=%s task_command_topic=%s",
            self.enable_stdin,
            self.input_topic,
            self.task_command_topic,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def publish_status(self, ok, message, payload=None):
        data = {"ok": bool(ok), "message": message}
        if payload:
            data.update(payload)
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=True)
        self.status_pub.publish(msg)
        if ok:
            rospy.loginfo(message)
        else:
            rospy.logwarn(message)

    def wait_for_task_subscriber(self):
        if not self.wait_for_executor:
            return True
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.task_pub.get_num_connections() > 0:
                return True
            if time.time() - start >= self.wait_timeout:
                return False
            rate.sleep()
        return False

    def handle_text(self, text, source):
        try:
            task = parse_task_text(text)
        except Exception as exc:
            self.publish_status(
                False,
                "parse failed from %s: %s" % (source, exc),
                {"source": source, "raw_text": to_unicode(text)},
            )
            return False

        if not self.wait_for_task_subscriber():
            self.publish_status(
                False,
                "no subscriber on %s; start grasp_executor first"
                % self.task_command_topic,
                {"source": source, "task": task},
            )
            return False

        msg = String()
        msg.data = json.dumps(task, ensure_ascii=True)
        self.task_pub.publish(msg)
        self.publish_status(
            True,
            "parsed task from %s: pick %d -> grid %d"
            % (source, task["pick_digit"], task["place_grid"]),
            {"source": source, "task": task},
        )
        return True

    def topic_callback(self, msg):
        self.handle_text(msg.data, "topic:%s" % self.input_topic)

    def stdin_loop(self):
        try:
            input_func = raw_input
        except NameError:
            input_func = input
        print("Text task interface ready. Example: 抓取 3 号方块放到 5 号格子")
        while not rospy.is_shutdown():
            try:
                line = input_func("> ")
            except EOFError:
                rospy.loginfo("stdin closed; topic input remains active")
                return
            if not line:
                continue
            command = normalize_text(line)
            if command.lower() in ("q", "quit", "exit"):
                rospy.signal_shutdown("user exit")
                return
            self.handle_text(command, "stdin")

    def run(self):
        if self.argv_text:
            self.handle_text(self.argv_text, "argv")
            rospy.sleep(0.5)
            return

        if self.enable_stdin:
            thread = threading.Thread(target=self.stdin_loop)
            thread.daemon = True
            thread.start()

        rospy.spin()


def main():
    argv = rospy.myargv(argv=sys.argv)[1:]
    try:
        TextTaskInterface(argv).run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
