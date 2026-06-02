#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import csv
import json
import os
import threading
import time

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty

from ur5_notebook.msg import GraspTargetArray


class ExperimentRecorder(object):
    """Run repeated grasp experiments and write trial-level metrics.

    Metrics:
    - recognition_success: target class appears as a stable localization target.
    - planning_success: grasp_executor reaches a post-planning execution phase.
    - grasp_success: grasp_executor reports state == "done".
    - task_time_sec: wall time from trial start to terminal status/timeout.
    """

    EXECUTION_PHASES = set(
        [
            "descending",
            "contacting",
            "vacuum_on",
            "retreating",
            "placing",
            "returning_home",
            "done",
        ]
    )
    TERMINAL_STATES = set(["done", "failed", "rejected", "timeout"])

    def __init__(self):
        rospy.init_node("experiment_recorder", anonymous=False)

        self.repetitions = int(rospy.get_param("~repetitions", 5))
        self.target_class_id = int(rospy.get_param("~target_class_id", 6))
        self.targets_topic = rospy.get_param(
            "~targets_topic", "/localization/grasp_targets"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/grasp/target_class_id"
        )
        self.status_topic = rospy.get_param("~status_topic", "/grasp/status")
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(
                os.path.expanduser("~"), "arm_ws", "experiment_results"
            ),
        )
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 20.0))
        self.execution_timeout = float(rospy.get_param("~execution_timeout", 120.0))
        self.inter_trial_delay = float(rospy.get_param("~inter_trial_delay", 3.0))
        self.reset_between_trials = self.as_bool(
            rospy.get_param("~reset_between_trials", False)
        )
        self.stop_before_reset = self.as_bool(
            rospy.get_param("~stop_before_reset", True)
        )
        self.poll_hz = float(rospy.get_param("~poll_hz", 10.0))

        self.lock = threading.Lock()
        self.latest_targets = {}
        self.latest_status = None
        self.latest_status_ts = 0.0
        self.status_events = []

        self.command_pub = rospy.Publisher(self.command_topic, Int32, queue_size=1)
        rospy.Subscriber(
            self.targets_topic, GraspTargetArray, self.targets_callback, queue_size=1
        )
        rospy.Subscriber(self.status_topic, String, self.status_callback, queue_size=10)

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def targets_callback(self, msg):
        targets = {}
        for target in msg.targets:
            if bool(target.stable):
                targets[int(target.class_id)] = {
                    "confidence": float(target.confidence),
                    "x": float(target.pose_base.position.x),
                    "y": float(target.pose_base.position.y),
                    "z": float(target.pose_base.position.z),
                }
        with self.lock:
            self.latest_targets = targets

    def status_callback(self, msg):
        ts = time.time()
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"state": "unknown", "message": msg.data}
        payload["_ts"] = ts
        with self.lock:
            self.latest_status = payload
            self.latest_status_ts = ts
            self.status_events.append(dict(payload))

    def stable_target_snapshot(self):
        with self.lock:
            return dict(self.latest_targets)

    def status_events_after(self, since_ts):
        with self.lock:
            return [dict(event) for event in self.status_events if event["_ts"] > since_ts]

    def wait_for_stable_target(self, class_id):
        deadline = time.time() + self.detection_timeout
        rate = rospy.Rate(self.poll_hz)
        last_snapshot = {}
        while time.time() < deadline and not rospy.is_shutdown():
            last_snapshot = self.stable_target_snapshot()
            if class_id in last_snapshot:
                return True, last_snapshot
            rate.sleep()
        return False, last_snapshot

    def wait_for_terminal_status(self, since_ts):
        deadline = time.time() + self.execution_timeout
        rate = rospy.Rate(self.poll_hz)
        planning_success = False
        terminal = None
        events = []

        while time.time() < deadline and not rospy.is_shutdown():
            events = self.status_events_after(since_ts)
            for event in events:
                state = event.get("state", "")
                if state in self.EXECUTION_PHASES:
                    planning_success = True
                if state in self.TERMINAL_STATES:
                    terminal = event
            if terminal is not None:
                return terminal, planning_success, events
            rate.sleep()

        timeout_event = {
            "state": "timeout",
            "message": "timeout waiting for terminal /grasp/status",
            "_ts": time.time(),
        }
        return timeout_event, planning_success, events

    def call_empty_service(self, name, timeout):
        try:
            rospy.wait_for_service(name, timeout=timeout)
            proxy = rospy.ServiceProxy(name, Empty)
            proxy()
            return True
        except Exception as exc:
            rospy.logwarn("service %s failed or unavailable: %s", name, exc)
            return False

    def reset_for_next_trial(self):
        if not self.reset_between_trials:
            return
        if self.stop_before_reset:
            self.call_empty_service("/grasp_executor/stop", 3.0)
        self.call_empty_service("/gazebo/reset_world", 5.0)

    def run_trial(self, trial_index):
        trial_start = time.time()
        rospy.loginfo(
            "trial %d/%d: waiting for stable class %d",
            trial_index,
            self.repetitions,
            self.target_class_id,
        )

        recognized, snapshot = self.wait_for_stable_target(self.target_class_id)
        if not recognized:
            elapsed = time.time() - trial_start
            return {
                "trial": trial_index,
                "target_class_id": self.target_class_id,
                "recognition_success": False,
                "planning_success": False,
                "grasp_success": False,
                "task_time_sec": elapsed,
                "terminal_state": "detection_timeout",
                "terminal_message": "stable target not observed",
                "stable_targets": snapshot,
                "status_events": [],
            }

        command_ts = time.time()
        cmd = Int32()
        cmd.data = self.target_class_id
        self.command_pub.publish(cmd)
        rospy.loginfo("trial %d: command sent class_id=%d", trial_index, cmd.data)

        terminal, planning_success, events = self.wait_for_terminal_status(command_ts)
        elapsed = time.time() - trial_start
        terminal_state = terminal.get("state", "unknown")
        terminal_message = terminal.get("message", "")
        grasp_success = terminal_state == "done"

        return {
            "trial": trial_index,
            "target_class_id": self.target_class_id,
            "recognition_success": True,
            "planning_success": bool(planning_success or grasp_success),
            "grasp_success": bool(grasp_success),
            "task_time_sec": elapsed,
            "terminal_state": terminal_state,
            "terminal_message": terminal_message,
            "stable_targets": snapshot,
            "status_events": events,
        }

    def summarize(self, trials):
        total = float(len(trials)) if trials else 1.0
        task_times = [float(t["task_time_sec"]) for t in trials]
        return {
            "repetitions": len(trials),
            "target_class_id": self.target_class_id,
            "recognition_success_rate": sum(1 for t in trials if t["recognition_success"]) / total,
            "planning_success_rate": sum(1 for t in trials if t["planning_success"]) / total,
            "grasp_success_rate": sum(1 for t in trials if t["grasp_success"]) / total,
            "average_task_time_sec": sum(task_times) / total if task_times else 0.0,
        }

    def write_outputs(self, trials, summary):
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, "experiment_%s.csv" % stamp)
        json_path = os.path.join(self.output_dir, "experiment_%s.json" % stamp)

        fieldnames = [
            "trial",
            "target_class_id",
            "recognition_success",
            "planning_success",
            "grasp_success",
            "task_time_sec",
            "terminal_state",
            "terminal_message",
        ]
        with open(csv_path, "w") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for trial in trials:
                writer.writerow({key: trial.get(key, "") for key in fieldnames})

        with open(json_path, "w") as json_file:
            json.dump(
                {"summary": summary, "trials": trials},
                json_file,
                ensure_ascii=True,
                indent=2,
                sort_keys=True,
            )

        return csv_path, json_path

    def run(self):
        trials = []
        rospy.sleep(1.0)
        for trial_index in range(1, self.repetitions + 1):
            if rospy.is_shutdown():
                break
            result = self.run_trial(trial_index)
            trials.append(result)
            rospy.loginfo(
                "trial %d result: recognition=%s planning=%s grasp=%s time=%.2fs state=%s",
                trial_index,
                result["recognition_success"],
                result["planning_success"],
                result["grasp_success"],
                result["task_time_sec"],
                result["terminal_state"],
            )
            if trial_index < self.repetitions:
                self.reset_for_next_trial()
                rospy.sleep(self.inter_trial_delay)

        summary = self.summarize(trials)
        csv_path, json_path = self.write_outputs(trials, summary)
        text = json.dumps(summary, ensure_ascii=True, sort_keys=True, indent=2)
        rospy.loginfo("experiment summary:\n%s", text)
        print(text)
        print("CSV: %s" % csv_path)
        print("JSON: %s" % json_path)


def main():
    try:
        ExperimentRecorder().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
