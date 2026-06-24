#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import json
import math
import sys
import threading
import time

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from ur5_notebook.msg import GraspTargetArray


class TaskManager(object):
    """
    E module task_manager

    Responsibilities:
    - Orchestrate step-by-step pick & place by driving grasp_executor via:
      /grasp/target_class_id + /grasp/status
    - Read stable grasp targets from:
      /localization/grasp_targets
    - Optionally select next task with a simple priority/distance scheduler
    - Global failure: stop remaining tasks and print failure report.
    """

    def __init__(self):
        rospy.init_node("task_manager", anonymous=False)

        # Topics
        self.targets_topic = rospy.get_param(
            "~targets_topic", "/localization/grasp_targets"
        )
        self.command_topic = rospy.get_param(
            "~command_topic", "/grasp/target_class_id"
        )
        self.task_command_topic = rospy.get_param(
            "~task_command_topic", "/grasp/task_command"
        )
        self.status_topic = rospy.get_param("~status_topic", "/grasp/status")
        self.use_structured_task_command = self.as_bool(
            rospy.get_param("~use_structured_task_command", True)
        )

        # Task table (loaded via rosparam YAML into this node namespace)
        # YAML format:
        # task_table:
        #   - order: 1
        #     pick_digit: 1
        #     place_grid: 1
        #     priority: 10
        self.task_table = rospy.get_param("~task_table", [])
        if not isinstance(self.task_table, list):
            rospy.logfatal("~task_table must be a list, got: %s", type(self.task_table))
            raise RuntimeError("Invalid task_table")

        # Scheduling / extensions
        self.require_stable = self.as_bool(rospy.get_param("~require_stable", True))
        self.enable_extension1_multi = self.as_bool(
            rospy.get_param("~enable_extension1_multi", True)
        )
        self.enable_extension7_scheduler = self.as_bool(
            rospy.get_param("~enable_extension7_scheduler", False)
        )
        self.alpha = float(rospy.get_param("~alpha", 0.6))  # priority - alpha * distance
        self.unreachable_policy = str(
            rospy.get_param("~unreachable_policy", "fail")
        ).strip().lower()
        if self.unreachable_policy not in ("fail", "skip"):
            rospy.logwarn(
                "Unknown unreachable_policy=%s, fallback to fail",
                self.unreachable_policy,
            )
            self.unreachable_policy = "fail"

        # Reachability precheck (aligned with grasp_executor defaults)
        self.reach_radius_xy = float(rospy.get_param("~reach_radius_xy", 0.97))
        self.require_place_reachable = self.as_bool(
            rospy.get_param("~require_place_reachable", True)
        )
        self.grid_origin_x = float(rospy.get_param("~grid_origin_x", -0.25))
        self.grid_origin_y = float(rospy.get_param("~grid_origin_y", -0.22))
        self.grid_spacing_x = float(rospy.get_param("~grid_spacing_x", 0.18))
        self.grid_spacing_y = float(rospy.get_param("~grid_spacing_y", 0.18))
        self.grid_cols = int(rospy.get_param("~grid_cols", 3))
        self.spawn_offset_x = float(rospy.get_param("~spawn_offset_x", 0.0))
        self.spawn_offset_y = float(rospy.get_param("~spawn_offset_y", 0.7))

        # Timeouts / polling
        self.stable_wait_timeout = float(rospy.get_param("~stable_wait_timeout", 20.0))
        self.status_wait_timeout = float(rospy.get_param("~status_wait_timeout", 90.0))
        self.poll_hz = float(rospy.get_param("~poll_hz", 10.0))
        self.max_total_timeout = float(rospy.get_param("~max_total_timeout", 0.0))
        # 连抓时上一段运动结束后控制器/PID 可能仍在收敛；下一步立刻下发
        # 容易导致 approach abort。两步之间等待若干秒再发下一抓取命令。
        self.inter_grasp_delay = float(rospy.get_param("~inter_grasp_delay", 0.0))

        self.status_sent_ts = None

        # Latest state
        self.lock = threading.Lock()
        self.latest_stable_targets = {}  # class_id -> GraspTarget-like object
        self.latest_all_targets = {}  # class_id -> GraspTarget-like object
        self.latest_status_payload = None
        self.latest_status_ts = 0.0

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        # Note: we don't publish status; grasp_executor owns /grasp/status.

        rospy.Subscriber(self.targets_topic, GraspTargetArray, self.targets_cb, queue_size=1)
        rospy.Subscriber(self.status_topic, String, self.status_cb, queue_size=1)

        # Build steps queue
        self.steps = self.parse_task_table(self.task_table)
        if not self.steps:
            rospy.logfatal("No steps found in ~task_table")
            raise RuntimeError("Empty steps")

        if not self.enable_extension1_multi:
            self.steps = self.steps[:1]

        # pending_steps contains indices of steps we haven't executed yet
        self.pending = list(range(len(self.steps)))
        if not self.enable_extension7_scheduler:
            # Default: execute in task table order (sorted by order field if present)
            self.pending.sort(key=lambda i: (self.steps[i].get("order_key", i), i))
        self.skipped_steps = []

        rospy.loginfo(
            "task_manager started: targets=%s command=%s task_command=%s structured=%s status=%s steps=%d multi=%s scheduler=%s",
            self.targets_topic,
            self.command_topic,
            self.task_command_topic,
            str(self.use_structured_task_command),
            self.status_topic,
            len(self.steps),
            str(self.enable_extension1_multi),
            str(self.enable_extension7_scheduler),
        )
        rospy.loginfo(
            "task_manager policy: unreachable_policy=%s reach_radius_xy=%.3f place_reachable=%s inter_grasp_delay=%.1fs",
            self.unreachable_policy,
            self.reach_radius_xy,
            str(self.require_place_reachable),
            self.inter_grasp_delay,
        )

        self.command_pub = rospy.Publisher(self.command_topic, Int32, queue_size=1)
        self.task_command_pub = rospy.Publisher(
            self.task_command_topic, String, queue_size=1
        )
        self.run()

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def parse_task_table(self, task_table):
        steps = []
        for idx, row in enumerate(task_table):
            if not isinstance(row, dict):
                rospy.logwarn("Skip non-dict step at index %d: %s", idx, type(row))
                continue
            try:
                pick_digit = int(row.get("pick_digit"))
                place_grid = int(row.get("place_grid")) if "place_grid" in row else pick_digit
            except Exception:
                rospy.logwarn("Invalid pick_digit/place_grid at index %d: %s", idx, row)
                continue

            order = row.get("order", None)
            priority = row.get("priority", 0)

            # Build an order key for sorting; if order is missing use YAML order.
            order_key = int(order) if order is not None else idx
            steps.append(
                {
                    "idx": idx,
                    "order": order,
                    "order_key": order_key,
                    "pick_digit": pick_digit,
                    "place_grid": place_grid,
                    "priority": float(priority),
                }
            )
        return steps

    def targets_cb(self, msg):
        stable_map = {}
        all_map = {}
        for t in msg.targets:
            cid = int(t.class_id)
            all_map[cid] = t
            if self.require_stable and not bool(t.stable):
                continue
            stable_map[cid] = t
        with self.lock:
            self.latest_stable_targets = stable_map
            self.latest_all_targets = all_map

    def status_cb(self, msg):
        ts = time.time()
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"state": "unknown", "message": msg.data}
        with self.lock:
            self.latest_status_payload = payload
            self.latest_status_ts = ts

    def get_stable_targets_copy(self):
        with self.lock:
            return dict(self.latest_stable_targets)

    def get_all_targets_copy(self):
        with self.lock:
            return dict(self.latest_all_targets)

    def get_status_after(self, since_ts):
        with self.lock:
            if self.latest_status_ts <= since_ts:
                return None
            return dict(self.latest_status_payload) if self.latest_status_payload else None

    def build_failure_snapshot(self, stable_targets):
        # stable class_id/confidence/yaw snapshot
        items = []
        for cid, t in stable_targets.items():
            items.append(
                {
                    "class_id": int(cid),
                    "confidence": float(getattr(t, "confidence", 0.0)),
                    "yaw_deg": float(getattr(t, "yaw_deg", 0.0)),
                }
            )
        items.sort(key=lambda x: x["confidence"], reverse=True)
        return items[:6]

    def classify_failure_type(self, payload):
        if not payload:
            return "规划失败"
        state = payload.get("state", "")
        msg = payload.get("message", "") or ""
        if state == "rejected":
            # grasp_executor uses rejected for unsupported/unreachable/slot missing
            if "grid slot" in msg or "no grid slot" in msg or "placeable" in msg:
                return "不可放置失败"
            if "out of UR5 reach" in msg or "unreachable" in msg:
                return "不可放置失败"
            return "不可放置失败"
        if state == "failed":
            lower = msg.lower()
            if "planning" in lower or "plan" in lower or "cartesian" in lower:
                return "规划失败"
            if "vacuum" in lower or "attach" in lower or "execution" in lower or "controller" in lower:
                return "抓取失败"
            return "抓取失败"
        return "规划失败"

    def mapping_is_compatible(self, step):
        pick_digit = int(step["pick_digit"])
        place_grid = int(step["place_grid"])

        if pick_digit < 0 or pick_digit > 9:
            return False, "pick_digit out of [0..9]"
        if place_grid < 1 or place_grid > 6:
            return False, "place_grid out of [1..6]"
        return True, ""

    def grid_world_xy(self, grid_id):
        index = int(grid_id) - 1
        wx = self.grid_origin_x + (index % self.grid_cols) * self.grid_spacing_x
        wy = self.grid_origin_y + (index // self.grid_cols) * self.grid_spacing_y
        return wx, wy

    def world_xy_to_base_xy(self, world_x, world_y):
        return world_x - self.spawn_offset_x, world_y - self.spawn_offset_y

    def is_target_reachable_xy(self, target):
        pos = target.pose_base.position
        return math.hypot(float(pos.x), float(pos.y)) <= self.reach_radius_xy

    def is_place_reachable_xy(self, place_grid):
        world_x, world_y = self.grid_world_xy(place_grid)
        bx, by = self.world_xy_to_base_xy(world_x, world_y)
        return math.hypot(float(bx), float(by)) <= self.reach_radius_xy

    def evaluate_step_executability(self, step, stable_targets):
        ok, reason = self.mapping_is_compatible(step)
        if not ok:
            return False, "mapping_invalid", reason, None

        pick_digit = int(step["pick_digit"])
        if pick_digit not in stable_targets:
            return False, "stable_missing", "pick_digit not in current stable targets", None

        pick_target = stable_targets[pick_digit]
        if not self.is_target_reachable_xy(pick_target):
            return False, "pick_unreachable", "pick target out of reach radius", pick_target

        if self.require_place_reachable:
            place_grid = int(step["place_grid"])
            if not self.is_place_reachable_xy(place_grid):
                return False, "place_unreachable", "place grid out of reach radius", pick_target

        return True, "", "", pick_target

    def wait_for_stable_pick(self, pick_digit):
        deadline = time.time() + self.stable_wait_timeout
        last_snapshot = None
        while time.time() < deadline and not rospy.is_shutdown():
            stable_targets = self.get_stable_targets_copy()
            last_snapshot = stable_targets
            if pick_digit in stable_targets:
                return True, stable_targets
            time.sleep(1.0 / self.poll_hz)
        return False, last_snapshot or {}

    def wait_for_initial_targets(self):
        """Block until detection produces at least one stable target, or timeout.

        Without this warmup, task_manager may evaluate executability before the
        first /localization/grasp_targets message has been processed by the
        subscriber callback. With unreachable_policy=skip that races into
        skipping every step (reason_code=stable_missing) immediately, so the
        robot never moves even though detection is healthy.
        """
        deadline = time.time() + self.stable_wait_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self.get_stable_targets_copy():
                return True
            time.sleep(1.0 / self.poll_hz)
        return False

    def wait_for_grasp_done(self, expected_pick_digit, expected_place_grid, sent_ts):
        deadline = time.time() + self.status_wait_timeout
        while time.time() < deadline and not rospy.is_shutdown():
            payload = self.get_status_after(sent_ts)
            if payload is None:
                time.sleep(1.0 / self.poll_hz)
                continue

            state = payload.get("state", "")
            msg = payload.get("message", "")
            if state == "done":
                extra_pick = payload.get("pick_digit", payload.get("class_id", None))
                if extra_pick is not None and int(extra_pick) != int(expected_pick_digit):
                    return False, {
                        "type": "抓取失败",
                        "payload": payload,
                        "msg": "done for unexpected pick_digit",
                    }
                extra_place = payload.get("place_grid", None)
                if extra_place is not None and int(extra_place) != int(expected_place_grid):
                    return False, {
                        "type": "抓取失败",
                        "payload": payload,
                        "msg": "done for unexpected place_grid",
                    }
                return True, payload

            if state in ("failed", "rejected"):
                failure_type = self.classify_failure_type(payload)
                return False, {"type": failure_type, "payload": payload, "msg": msg}

            time.sleep(1.0 / self.poll_hz)

        # timeout
        return False, {
            "type": "规划失败",
            "payload": {"state": "timeout", "message": "timeout waiting /grasp/status done"},
            "msg": "timeout waiting /grasp/status done",
        }

    def report_failure(
        self,
        failure_type,
        step,
        step_idx_for_print,
        final_class_id,
        stable_targets_snapshot,
        pick_target,
        status_payload,
    ):
        pick_digit = int(step["pick_digit"])
        place_grid = int(step["place_grid"])

        stable_snapshot = self.build_failure_snapshot(stable_targets_snapshot)

        pose_base = None
        pose_world = None
        if pick_target is not None:
            pose_base = {
                "x": float(pick_target.pose_base.position.x),
                "y": float(pick_target.pose_base.position.y),
                "z": float(pick_target.pose_base.position.z),
            }
            pose_world = {
                "x": float(pick_target.pose_world.position.x),
                "y": float(pick_target.pose_world.position.y),
                "z": float(pick_target.pose_world.position.z),
            }

        state = None
        message = None
        if status_payload:
            state = status_payload.get("state", None)
            message = status_payload.get("message", None)

        report = {
            "failure_type": failure_type,
            "step": {
                "step_index": step_idx_for_print,
                "pick_digit": pick_digit,
                "place_grid": place_grid,
                "final_class_id": final_class_id,
            },
            "latest_recognition_snapshot_stable": stable_snapshot,
            "latest_target_pose": {
                "pose_base": pose_base,
                "pose_world": pose_world,
            },
            "grasp_status": {
                "state": state,
                "message": message,
            },
        }

        # Terminal output (required)
        text = json.dumps(report, ensure_ascii=True, sort_keys=True, indent=2)
        rospy.logerr("\n%s", text)
        sys.stdout.write(text + "\n")
        sys.stdout.flush()

    def report_skip(self, step, step_idx_for_print, reason_code, reason_text, stable_targets_snapshot, pick_target):
        report = {
            "event": "skipped",
            "reason_code": reason_code,
            "reason": reason_text,
            "step": {
                "step_index": step_idx_for_print,
                "pick_digit": int(step["pick_digit"]),
                "place_grid": int(step["place_grid"]),
            },
            "latest_recognition_snapshot_stable": self.build_failure_snapshot(stable_targets_snapshot),
            "latest_target_pose": {
                "pose_base": None if pick_target is None else {
                    "x": float(pick_target.pose_base.position.x),
                    "y": float(pick_target.pose_base.position.y),
                    "z": float(pick_target.pose_base.position.z),
                },
                "pose_world": None if pick_target is None else {
                    "x": float(pick_target.pose_world.position.x),
                    "y": float(pick_target.pose_world.position.y),
                    "z": float(pick_target.pose_world.position.z),
                },
            },
        }
        self.skipped_steps.append(report)
        text = json.dumps(report, ensure_ascii=True, sort_keys=True, indent=2)
        rospy.logwarn("\n%s", text)
        sys.stdout.write(text + "\n")
        sys.stdout.flush()

    def select_next_task(self):
        # Extension 7: pick among pending steps by score
        stable_targets = self.get_stable_targets_copy()
        candidates = []
        for i in list(self.pending):
            step = self.steps[i]
            ok, reason_code, reason_text, pick_target = self.evaluate_step_executability(
                step, stable_targets
            )
            if not ok:
                continue

            t = pick_target
            pos = t.pose_base.position
            distance = math.sqrt(float(pos.x) ** 2 + float(pos.y) ** 2 + float(pos.z) ** 2)
            score = float(step.get("priority", 0.0)) - self.alpha * distance
            candidates.append((score, i, distance))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0], reverse=True)
        return candidates[0][1]

    def execute_step(self, step, step_idx_for_print, expected_final_class_id):
        # 1) Mapping compatibility check (SEND-time prevention)
        ok, reason = self.mapping_is_compatible(step)
        if not ok:
            stable_targets_snapshot = self.get_stable_targets_copy()
            pick_target = stable_targets_snapshot.get(int(step["pick_digit"]))
            self.report_failure(
                failure_type="不可放置失败",
                step=step,
                step_idx_for_print=step_idx_for_print,
                final_class_id=None,
                stable_targets_snapshot=stable_targets_snapshot,
                pick_target=pick_target,
                status_payload=self.latest_status_payload,
            )
            return False

        pick_digit = int(step["pick_digit"])
        place_grid = int(step["place_grid"])

        # 2) WAIT_STABLE_TARGET
        ok_stable, stable_targets_snapshot = self.wait_for_stable_pick(pick_digit)
        if not ok_stable:
            pick_target = stable_targets_snapshot.get(pick_digit) if stable_targets_snapshot else None
            self.report_failure(
                failure_type="检测失败",
                step=step,
                step_idx_for_print=step_idx_for_print,
                final_class_id=expected_final_class_id,
                stable_targets_snapshot=stable_targets_snapshot,
                pick_target=pick_target,
                status_payload=self.latest_status_payload,
            )
            return False

        pick_target = stable_targets_snapshot.get(pick_digit, None)

        # 3) SEND_PICK_AND_PLACE_COMMAND
        sent_ts = time.time()
        if self.use_structured_task_command:
            cmd = String()
            cmd.data = json.dumps(
                {
                    "pick_digit": pick_digit,
                    "place_grid": place_grid,
                    "source": "task_manager",
                },
                ensure_ascii=True,
            )
            self.task_command_pub.publish(cmd)
        else:
            if pick_digit != place_grid:
                self.report_failure(
                    failure_type="不可放置失败",
                    step=step,
                    step_idx_for_print=step_idx_for_print,
                    final_class_id=expected_final_class_id,
                    stable_targets_snapshot=stable_targets_snapshot,
                    pick_target=pick_target,
                    status_payload={
                        "state": "rejected",
                        "message": "legacy Int32 command requires pick_digit == place_grid",
                    },
                )
                return False
            cmd = Int32()
            cmd.data = int(expected_final_class_id)
            self.command_pub.publish(cmd)

        # 4) WAIT_GRASP_STATUS_DONE
        ok_done, payload_or_err = self.wait_for_grasp_done(
            pick_digit, place_grid, sent_ts
        )
        if not ok_done:
            failure_type = payload_or_err.get("type", "抓取失败")
            status_payload = payload_or_err.get("payload", None)
            self.report_failure(
                failure_type=failure_type,
                step=step,
                step_idx_for_print=step_idx_for_print,
                final_class_id=expected_final_class_id,
                stable_targets_snapshot=stable_targets_snapshot,
                pick_target=pick_target,
                status_payload=status_payload,
            )
            return False

        return True

    def run(self):
        start_ts = time.time()
        executed = 0

        rospy.loginfo("task_manager IDLE -> BUILD_TASK_QUEUE")
        # Warmup: make sure detection has published at least one stable target
        # before we start evaluating executability, otherwise skip policy would
        # skip everything on the first (empty) snapshot.
        if self.wait_for_initial_targets():
            rospy.loginfo("task_manager: initial stable targets received, starting scheduling")
        else:
            rospy.logwarn(
                "task_manager: no stable targets within %.1fs warmup; proceeding (steps may be skipped)",
                self.stable_wait_timeout,
            )
        while not rospy.is_shutdown():
            if self.max_total_timeout > 0 and time.time() - start_ts > self.max_total_timeout:
                rospy.logerr("task_manager timeout: max_total_timeout reached")
                rospy.signal_shutdown("max_total_timeout")
                return

            if not self.pending:
                rospy.loginfo("All steps done.")
                rospy.signal_shutdown("completed")
                return

            # SELECT_NEXT_TASK (extension 7 trigger)
            if self.enable_extension7_scheduler:
                stable_targets_snapshot = self.get_stable_targets_copy()
                # For skip policy: first drop all currently non-executable steps in this round.
                if self.unreachable_policy == "skip":
                    still_pending = []
                    for i in list(self.pending):
                        step = self.steps[i]
                        ok, reason_code, reason_text, pick_target = self.evaluate_step_executability(
                            step, stable_targets_snapshot
                        )
                        if ok:
                            still_pending.append(i)
                        else:
                            self.report_skip(
                                step=step,
                                step_idx_for_print=step["idx"],
                                reason_code=reason_code,
                                reason_text=reason_text,
                                stable_targets_snapshot=stable_targets_snapshot,
                                pick_target=pick_target,
                            )
                    self.pending = still_pending
                    if not self.pending:
                        rospy.loginfo(
                            "All executable steps finished. skipped=%d",
                            len(self.skipped_steps),
                        )
                        rospy.signal_shutdown("completed_with_skips")
                        return

                chosen_index = self.select_next_task()
                if chosen_index is None:
                    # No executable candidates based on current stable targets
                    if self.unreachable_policy == "skip":
                        rospy.loginfo(
                            "No executable candidates now (skip policy). pending=%d skipped=%d",
                            len(self.pending),
                            len(self.skipped_steps),
                        )
                        rospy.signal_shutdown("completed_with_skips")
                        return
                    # fail policy: keep previous global-fail behavior
                    # We pick the next expected step idx for printing if available.
                    step = self.steps[self.pending[0]]
                    stable_targets_snapshot = self.get_stable_targets_copy()
                    self.report_failure(
                        failure_type="检测失败",
                        step=step,
                        step_idx_for_print=step["idx"],
                        final_class_id=None,
                        stable_targets_snapshot=stable_targets_snapshot,
                        pick_target=stable_targets_snapshot.get(int(step["pick_digit"])),
                        status_payload=self.latest_status_payload,
                    )
                    rospy.signal_shutdown("no executable candidates")
                    return

                current_step = self.steps[chosen_index]
                # remove from pending immediately
                self.pending.remove(chosen_index)
                step_idx_for_print = current_step["idx"]
            else:
                # default: sequential by queue order
                current_step = None
                step_idx_for_print = None
                while self.pending and current_step is None:
                    chosen_index = self.pending.pop(0)
                    candidate = self.steps[chosen_index]
                    stable_targets_snapshot = self.get_stable_targets_copy()
                    ok, reason_code, reason_text, pick_target = self.evaluate_step_executability(
                        candidate, stable_targets_snapshot
                    )
                    # The pick digit may simply not be stable yet (detection still
                    # confirming). Give it a bounded chance before skipping/failing.
                    if (not ok) and reason_code == "stable_missing":
                        self.wait_for_stable_pick(int(candidate["pick_digit"]))
                        stable_targets_snapshot = self.get_stable_targets_copy()
                        ok, reason_code, reason_text, pick_target = self.evaluate_step_executability(
                            candidate, stable_targets_snapshot
                        )
                    if ok:
                        current_step = candidate
                        step_idx_for_print = candidate["idx"]
                        break
                    if self.unreachable_policy == "skip":
                        self.report_skip(
                            step=candidate,
                            step_idx_for_print=candidate["idx"],
                            reason_code=reason_code,
                            reason_text=reason_text,
                            stable_targets_snapshot=stable_targets_snapshot,
                            pick_target=pick_target,
                        )
                        continue
                    # fail policy
                    self.report_failure(
                        failure_type="不可放置失败" if reason_code == "mapping_invalid" else "检测失败",
                        step=candidate,
                        step_idx_for_print=candidate["idx"],
                        final_class_id=None,
                        stable_targets_snapshot=stable_targets_snapshot,
                        pick_target=pick_target,
                        status_payload=self.latest_status_payload,
                    )
                    rospy.signal_shutdown("step_not_executable")
                    return
                if current_step is None:
                    rospy.loginfo(
                        "No executable steps left. skipped=%d",
                        len(self.skipped_steps),
                    )
                    rospy.signal_shutdown("completed_with_skips")
                    return

            # Structured command path supports pick_digit and place_grid separately.
            expected_final_class_id = int(current_step["place_grid"])

            rospy.loginfo(
                "Executing step: step_idx=%d pick_digit=%d place_grid=%d",
                step_idx_for_print,
                int(current_step["pick_digit"]),
                int(current_step["place_grid"]),
            )

            ok = self.execute_step(current_step, step_idx_for_print, expected_final_class_id)
            if not ok:
                # execute_step already printed failure report
                return

            executed += 1
            rospy.loginfo("Step finished: step_idx=%d executed=%d", step_idx_for_print, executed)

            if self.pending and self.inter_grasp_delay > 0:
                rospy.loginfo(
                    "inter_grasp_delay: waiting %.1fs for controller settle before next step",
                    self.inter_grasp_delay,
                )
                rospy.sleep(self.inter_grasp_delay)


def main():
    try:
        TaskManager()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
