#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import sys

import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from ur5_notebook.msg import Detection2DArray
from ur5_notebook.msg import GraspTarget
from ur5_notebook.msg import GraspTargetArray


class BlockLocalization(object):
    """Convert stable 2D detections into 3D grasp poses in world and base_link."""

    def __init__(self):
        rospy.init_node("block_localization", anonymous=False)

        self.detection_topic = rospy.get_param(
            "~detection_topic", "/detection/results"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/ur5/wrist_camera/camera_info"
        )
        self.targets_topic = rospy.get_param(
            "~targets_topic", "/localization/grasp_targets"
        )
        self.debug_topic = rospy.get_param("~debug_topic", "/localization/debug")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.require_stable = self.as_bool(rospy.get_param("~require_stable", True))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 1.0))

        self.table_z = float(rospy.get_param("~table_z", 0.10))
        self.table_size_z = float(rospy.get_param("~table_size_z", 0.20))
        self.block_size_z = float(rospy.get_param("~block_size_z", 0.04))
        self.block_surface_offset = float(
            rospy.get_param("~block_surface_offset", 0.002)
        )
        self.grasp_height_offset = float(
            rospy.get_param("~grasp_height_offset", 0.0)
        )

        self.view_margin_px = float(rospy.get_param("~view_margin_px", 30.0))
        self.view_bounds = self.build_virtual_view_bounds()
        self.block_center_z = (
            self.table_z
            + self.table_size_z * 0.5
            + self.block_size_z * 0.5
            + self.block_surface_offset
            + self.grasp_height_offset
        )

        self.camera_width = int(rospy.get_param("~camera_width", 1280))
        self.camera_height = int(rospy.get_param("~camera_height", 960))
        self.tf_listener = tf.TransformListener()
        self.tf_warning_time = 0.0

        self.targets_pub = rospy.Publisher(
            self.targets_topic, GraspTargetArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=1)
        rospy.Subscriber(
            self.detection_topic,
            Detection2DArray,
            self.detection_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "block_localization started: detection=%s world=%s base=%s block_center_z=%.4f",
            self.detection_topic,
            self.world_frame,
            self.base_frame,
            self.block_center_z,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def build_virtual_view_bounds(self):
        table_x = float(rospy.get_param("~table_x", 0.25))
        table_y = float(rospy.get_param("~table_y", 0.0))
        table_size_x = float(rospy.get_param("~table_size_x", 1.2))
        table_size_y = float(rospy.get_param("~table_size_y", 0.9))
        grid_cell_size = float(rospy.get_param("~grid_cell_size", 0.13))
        grid_origin_x = float(rospy.get_param("~grid_origin_x", -0.25))
        grid_origin_y = float(rospy.get_param("~grid_origin_y", -0.22))
        grid_spacing_x = float(rospy.get_param("~grid_spacing_x", 0.18))
        grid_spacing_y = float(rospy.get_param("~grid_spacing_y", 0.18))
        grid_cols = int(rospy.get_param("~grid_cols", 3))
        grid_rows = int(rospy.get_param("~grid_rows", 2))
        draw_target_grid = self.as_bool(rospy.get_param("~draw_target_grid", True))

        table_x_min = table_x - table_size_x * 0.5
        table_x_max = table_x + table_size_x * 0.5
        table_y_min = table_y - table_size_y * 0.5
        table_y_max = table_y + table_size_y * 0.5
        spawn_x_min = float(rospy.get_param("~spawn_x_min", 0.34))
        spawn_x_max = float(rospy.get_param("~spawn_x_max", 0.75))
        spawn_y_min = float(rospy.get_param("~spawn_y_min", -0.30))
        spawn_y_max = float(rospy.get_param("~spawn_y_max", 0.28))
        grid_x_min = grid_origin_x - grid_cell_size * 0.5
        grid_x_max = grid_origin_x + (grid_cols - 1) * grid_spacing_x + grid_cell_size * 0.5
        grid_y_min = grid_origin_y - grid_cell_size * 0.5
        grid_y_max = grid_origin_y + (grid_rows - 1) * grid_spacing_y + grid_cell_size * 0.5
        default_x_min = min(spawn_x_min, grid_x_min if draw_target_grid else spawn_x_min)
        default_x_max = max(spawn_x_max, grid_x_max if draw_target_grid else spawn_x_max)
        default_y_min = min(spawn_y_min, grid_y_min if draw_target_grid else spawn_y_min)
        default_y_max = max(spawn_y_max, grid_y_max if draw_target_grid else spawn_y_max)
        view_padding = float(rospy.get_param("~view_padding", 0.08))
        x_min = max(
            table_x_min,
            float(rospy.get_param("~view_x_min", default_x_min - view_padding)),
        )
        x_max = min(
            table_x_max,
            float(rospy.get_param("~view_x_max", default_x_max + view_padding)),
        )
        y_min = max(
            table_y_min,
            float(rospy.get_param("~view_y_min", default_y_min - view_padding)),
        )
        y_max = min(
            table_y_max,
            float(rospy.get_param("~view_y_max", default_y_max + view_padding)),
        )
        return x_min, x_max, y_min, y_max

    def camera_info_callback(self, msg):
        if msg.width > 0 and msg.height > 0:
            self.camera_width = int(msg.width)
            self.camera_height = int(msg.height)

    def pixel_to_world(self, u, v):
        width = float(self.camera_width)
        height = float(self.camera_height)
        x_min, x_max, y_min, y_max = self.view_bounds
        usable_w = width - 2.0 * self.view_margin_px
        usable_h = height - 2.0 * self.view_margin_px
        if usable_w <= 1.0 or usable_h <= 1.0:
            return None
        world_x = x_min + (float(u) - self.view_margin_px) * (x_max - x_min) / usable_w
        world_y = (
            y_min
            + (height - self.view_margin_px - float(v)) * (y_max - y_min) / usable_h
        )
        return world_x, world_y

    def make_world_pose(self, world_x, world_y, yaw_deg):
        pose = Pose()
        pose.position.x = world_x
        pose.position.y = world_y
        pose.position.z = self.block_center_z
        quat = tft.quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def transform_pose_to_base(self, world_pose, stamp):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = self.world_frame
        pose_stamped.pose = world_pose
        try:
            return self.tf_listener.transformPose(self.base_frame, pose_stamped)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            now = rospy.get_time()
            if now - self.tf_warning_time > 5.0:
                rospy.logwarn(
                    "TF transform %s -> %s unavailable: %s",
                    self.world_frame,
                    self.base_frame,
                    exc,
                )
                self.tf_warning_time = now
            return None

    def detection_callback(self, msg):
        targets = []
        debug_entries = []

        for det in msg.detections:
            if self.require_stable and not det.stable:
                continue

            world_xy = self.pixel_to_world(det.center_x, det.center_y)
            if world_xy is None:
                continue

            world_x, world_y = world_xy
            world_pose = self.make_world_pose(world_x, world_y, det.yaw_deg)
            base_pose_stamped = self.transform_pose_to_base(world_pose, msg.header.stamp)
            if base_pose_stamped is None:
                continue

            target = GraspTarget()
            target.class_id = int(det.class_id)
            target.class_name = str(det.class_name)
            target.confidence = float(det.confidence)
            target.stable = bool(det.stable)
            target.center_x = int(det.center_x)
            target.center_y = int(det.center_y)
            target.yaw_deg = float(det.yaw_deg)
            target.pose_world = world_pose
            target.pose_base = base_pose_stamped.pose
            targets.append(target)

            debug_entries.append(
                {
                    "class_id": target.class_id,
                    "class_name": target.class_name,
                    "confidence": target.confidence,
                    "stable": target.stable,
                    "pixel": [target.center_x, target.center_y],
                    "world_xy": [world_x, world_y],
                    "world_z": self.block_center_z,
                    "yaw_deg": target.yaw_deg,
                    "base_xyz": [
                        base_pose_stamped.pose.position.x,
                        base_pose_stamped.pose.position.y,
                        base_pose_stamped.pose.position.z,
                    ],
                }
            )

        array_msg = GraspTargetArray()
        array_msg.header = msg.header
        array_msg.header.frame_id = self.base_frame
        array_msg.targets = targets
        self.targets_pub.publish(array_msg)

        debug_msg = String()
        debug_msg.data = json.dumps(
            {
                "frame_id": self.base_frame,
                "target_count": len(targets),
                "camera_size": [self.camera_width, self.camera_height],
                "view_bounds": list(self.view_bounds),
                "targets": debug_entries,
            }
        )
        self.debug_pub.publish(debug_msg)


def main():
    try:
        BlockLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
