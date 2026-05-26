#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import sys
import threading
from copy import deepcopy

import moveit_commander
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from ur5_notebook.msg import GraspTarget
from ur5_notebook.msg import GraspTargetArray


DEFAULT_DOWN_ORIENTATION = (0.4811, 0.4994, -0.5121, 0.5069)
VACUUM_SERVICES = [
    "/ur5/vacuum_gripper",
    "/ur5/vacuum_gripper1",
    "/ur5/vacuum_gripper2",
    "/ur5/vacuum_gripper3",
    "/ur5/vacuum_gripper4",
    "/ur5/vacuum_gripper5",
    "/ur5/vacuum_gripper6",
    "/ur5/vacuum_gripper7",
    "/ur5/vacuum_gripper8",
]


class GraspExecutor(object):
    """Execute pick-and-place motions from localized grasp targets."""

    def __init__(self):
        rospy.init_node("grasp_executor", anonymous=False)

        self.targets_topic = rospy.get_param(
            "~targets_topic", "/localization/grasp_targets"
        )
        self.command_topic = rospy.get_param("~command_topic", "/grasp/target_class_id")
        self.status_topic = rospy.get_param("~status_topic", "/grasp/status")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.move_group = rospy.get_param("~move_group", "manipulator")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.use_all_vacuum_nodes = self.as_bool(
            rospy.get_param("~use_all_vacuum_nodes", True)
        )
        self.enable_place = self.as_bool(rospy.get_param("~enable_place", True))
        self.use_detection_yaw = self.as_bool(
            rospy.get_param("~use_detection_yaw", False)
        )
        self.require_stable = self.as_bool(
            rospy.get_param("~require_stable", True)
        )
        self.approach_z_offset = float(rospy.get_param("~approach_z_offset", 0.12))
        self.grasp_z_offset = float(rospy.get_param("~grasp_z_offset", 0.02))
        self.retreat_z_offset = float(rospy.get_param("~retreat_z_offset", 0.15))
        self.place_z_offset = float(rospy.get_param("~place_z_offset", 0.12))
        self.gripper_wait = float(rospy.get_param("~gripper_wait", 1.0))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.01)
        )
        self.orientation_tolerance = float(
            rospy.get_param("~orientation_tolerance", 0.15)
        )
        self.max_velocity_scaling = float(
            rospy.get_param("~max_velocity_scaling", 0.35)
        )
        self.max_acceleration_scaling = float(
            rospy.get_param("~max_acceleration_scaling", 0.35)
        )
        self.cartesian_min_fraction = float(
            rospy.get_param("~cartesian_min_fraction", 0.80)
        )
        self.moveit_startup_delay = float(
            rospy.get_param("~moveit_startup_delay", 3.0)
        )

        self.table_z = float(rospy.get_param("~table_z", 0.10))
        self.table_size_z = float(rospy.get_param("~table_size_z", 0.20))
        self.block_size_z = float(rospy.get_param("~block_size_z", 0.04))
        self.grid_origin_x = float(rospy.get_param("~grid_origin_x", -0.25))
        self.grid_origin_y = float(rospy.get_param("~grid_origin_y", -0.22))
        self.grid_spacing_x = float(rospy.get_param("~grid_spacing_x", 0.18))
        self.grid_spacing_y = float(rospy.get_param("~grid_spacing_y", 0.18))
        self.grid_cols = int(rospy.get_param("~grid_cols", 3))

        self.latest_targets = {}
        self.busy = False
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()

        rospy.on_shutdown(self.cleanup)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(self.moveit_startup_delay)

        self.arm = moveit_commander.MoveGroupCommander(self.move_group)
        self.end_effector_link = self.arm.get_end_effector_link()
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(self.position_tolerance)
        self.arm.set_goal_orientation_tolerance(self.orientation_tolerance)
        self.arm.set_planning_time(self.planning_time)
        self.arm.set_max_velocity_scaling_factor(self.max_velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.max_acceleration_scaling)

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        rospy.Subscriber(
            self.targets_topic,
            GraspTargetArray,
            self.targets_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.command_topic,
            Int32,
            self.command_callback,
            queue_size=1,
        )
        rospy.Service("~stop", Empty, self.stop_callback)

        self.publish_status("idle", "grasp_executor ready")
        rospy.loginfo(
            "grasp_executor ready: targets=%s command=%s ee=%s",
            self.targets_topic,
            self.command_topic,
            self.end_effector_link,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def cleanup(self):
        try:
            self.arm.stop()
        except Exception:
            pass
        moveit_commander.roscpp_shutdown()

    def publish_status(self, state, message, extra=None):
        payload = {"state": state, "message": message}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        rospy.loginfo("grasp_executor [%s]: %s", state, message)

    def targets_callback(self, msg):
        targets = {}
        for target in msg.targets:
            if self.require_stable and not target.stable:
                continue
            targets[int(target.class_id)] = target
        with self.lock:
            self.latest_targets = targets

    def command_callback(self, msg):
        class_id = int(msg.data)
        if class_id < 0 or class_id > 9:
            self.publish_status("rejected", "class_id must be in [0, 9]")
            return
        if self.busy:
            self.publish_status("busy", "grasp already in progress")
            return
        thread = threading.Thread(
            target=self.execute_grasp, args=(class_id,)
        )
        thread.daemon = True
        thread.start()

    def stop_callback(self, _req):
        try:
            self.arm.stop()
        except Exception:
            pass
        self.set_gripper(False)
        self.busy = False
        self.publish_status("stopped", "motion stopped and gripper released")
        return EmptyResponse()

    def get_target(self, class_id):
        with self.lock:
            return self.latest_targets.get(class_id)

    def build_tool_orientation(self, yaw_deg):
        if not self.use_detection_yaw:
            qx, qy, qz, qw = DEFAULT_DOWN_ORIENTATION
            return qx, qy, qz, qw
        quat = tft.quaternion_multiply(
            DEFAULT_DOWN_ORIENTATION,
            tft.quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg)),
        )
        return quat[0], quat[1], quat[2], quat[3]

    def make_pose(self, target, z_offset):
        pose = deepcopy(target.pose_base)
        pose.position.z += z_offset
        qx, qy, qz, qw = self.build_tool_orientation(target.yaw_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def transform_world_pose_to_base(self, world_pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.header.frame_id = self.world_frame
        pose_stamped.pose = world_pose
        try:
            transformed = self.tf_listener.transformPose(self.base_frame, pose_stamped)
            return transformed.pose
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn("Failed to transform place pose: %s", exc)
            return None

    def grid_pose_world(self, grid_id):
        index = int(grid_id) - 1
        pose = Pose()
        pose.position.x = self.grid_origin_x + (index % self.grid_cols) * self.grid_spacing_x
        pose.position.y = self.grid_origin_y + (index // self.grid_cols) * self.grid_spacing_y
        pose.position.z = (
            self.table_z + self.table_size_z * 0.5 + self.block_size_z * 0.5 + 0.001
        )
        qx, qy, qz, qw = DEFAULT_DOWN_ORIENTATION
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def set_gripper(self, enabled):
        suffix = "on" if enabled else "off"
        namespaces = VACUUM_SERVICES if self.use_all_vacuum_nodes else VACUUM_SERVICES[:1]
        for namespace in namespaces:
            service_name = "%s/%s" % (namespace, suffix)
            try:
                rospy.wait_for_service(service_name, timeout=2.0)
                proxy = rospy.ServiceProxy(service_name, Empty)
                proxy()
            except rospy.ServiceException as exc:
                rospy.logwarn("Gripper service %s failed: %s", service_name, exc)

    def move_to_pose(self, pose, label):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        if not plan:
            self.publish_status("failed", "%s planning failed" % label)
            return False
        success = self.arm.execute(plan, wait=True)
        if not success:
            self.publish_status("failed", "%s execution failed" % label)
            return False
        return True

    def move_cartesian(self, waypoints, label):
        self.arm.set_start_state_to_current_state()
        plan, fraction = self.arm.compute_cartesian_path(
            waypoints, 0.01, 0.0, True
        )
        if fraction < self.cartesian_min_fraction:
            self.publish_status(
                "failed",
                "%s cartesian path incomplete (fraction=%.2f)" % (label, fraction),
            )
            return False
        success = self.arm.execute(plan, wait=True)
        if not success:
            self.publish_status("failed", "%s cartesian execution failed" % label)
            return False
        return True

    def execute_grasp(self, class_id):
        self.busy = True
        try:
            target = self.get_target(class_id)
            if target is None:
                self.publish_status(
                    "failed",
                    "no stable grasp target for class %d" % class_id,
                    {"class_id": class_id},
                )
                return

            self.publish_status(
                "approaching",
                "moving to class %d" % class_id,
                {"class_id": class_id, "confidence": target.confidence},
            )

            approach_pose = self.make_pose(target, self.approach_z_offset)
            grasp_pose = self.make_pose(target, self.grasp_z_offset)
            retreat_pose = self.make_pose(target, self.retreat_z_offset)

            if not self.move_to_pose(approach_pose, "approach"):
                return

            if not self.move_cartesian(
                [deepcopy(grasp_pose)], "descend"
            ):
                return

            self.set_gripper(True)
            rospy.sleep(self.gripper_wait)

            if not self.move_cartesian(
                [deepcopy(retreat_pose)], "retreat"
            ):
                return

            placed = False
            if self.enable_place and 1 <= class_id <= 6:
                place_world = self.grid_pose_world(class_id)
                place_base = self.transform_world_pose_to_base(place_world)
                if place_base is not None:
                    place_approach = deepcopy(place_base)
                    place_approach.position.z += self.place_z_offset
                    place_touchdown = deepcopy(place_base)
                    place_retreat = deepcopy(place_base)
                    place_retreat.position.z += self.place_z_offset

                    self.publish_status(
                        "placing",
                        "moving digit %d to grid %d" % (class_id, class_id),
                        {"class_id": class_id},
                    )
                    if self.move_to_pose(place_approach, "place approach"):
                        if self.move_cartesian([place_touchdown], "place descend"):
                            self.set_gripper(False)
                            rospy.sleep(0.5)
                            self.move_cartesian([place_retreat], "place retreat")
                            placed = True

            if not placed:
                self.set_gripper(False)

            self.publish_status(
                "done",
                "grasp complete for class %d" % class_id,
                {"class_id": class_id, "placed": placed},
            )
        finally:
            self.busy = False


def main():
    try:
        GraspExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
