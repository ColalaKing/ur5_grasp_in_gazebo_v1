#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math

import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from ur5_notebook.msg import Detection2DArray


class GraspPoseConverter(object):
    """Convert 2-D detection centers into table-top grasp poses."""

    def __init__(self):
        rospy.init_node("grasp_pose_converter", anonymous=False)

        self.detections_topic = rospy.get_param("~detections_topic", "/detection/results")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/ur5/wrist_camera/camera_info"
        )
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.stable_only = self.as_bool(rospy.get_param("~stable_only", True))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.0))
        self.publish_all = self.as_bool(rospy.get_param("~publish_all", True))

        self.image_width = int(rospy.get_param("~image_width", 1280))
        self.image_height = int(rospy.get_param("~image_height", 960))
        self.view_margin_px = float(rospy.get_param("~view_margin_px", 30.0))

        self.table_z = float(rospy.get_param("~table_z", 0.10))
        self.table_size_z = float(rospy.get_param("~table_size_z", 0.20))
        self.block_size_z = float(rospy.get_param("~block_size_z", 0.04))
        self.grasp_z_offset = float(rospy.get_param("~grasp_z_offset", 0.015))
        self.approach_z_offset = float(rospy.get_param("~approach_z_offset", 0.10))

        self.view_bounds = self.build_virtual_view_bounds()
        self.tf_listener = tf.TransformListener()
        self.camera_info = None

        self.poses_pub = rospy.Publisher("/grasp_targets/poses", PoseArray, queue_size=1)
        self.selected_pub = rospy.Publisher("/grasp_targets/selected", PoseStamped, queue_size=1)
        self.approach_pub = rospy.Publisher("/grasp_targets/approach", PoseStamped, queue_size=1)
        self.details_pub = rospy.Publisher("/grasp_targets/details", String, queue_size=1)

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )
        rospy.Subscriber(
            self.detections_topic, Detection2DArray, self.detections_callback, queue_size=1
        )

        rospy.loginfo(
            "grasp_pose_converter started: detections=%s target_frame=%s stable_only=%s",
            self.detections_topic,
            self.target_frame,
            self.stable_only,
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

        x_min = max(table_x_min, float(rospy.get_param("~view_x_min", default_x_min - view_padding)))
        x_max = min(table_x_max, float(rospy.get_param("~view_x_max", default_x_max + view_padding)))
        y_min = max(table_y_min, float(rospy.get_param("~view_y_min", default_y_min - view_padding)))
        y_max = min(table_y_max, float(rospy.get_param("~view_y_max", default_y_max + view_padding)))
        return x_min, x_max, y_min, y_max

    def camera_info_callback(self, msg):
        self.camera_info = msg
        if msg.width:
            self.image_width = int(msg.width)
        if msg.height:
            self.image_height = int(msg.height)

    def detections_callback(self, msg):
        candidates = []
        for det in msg.detections:
            if self.stable_only and not det.stable:
                continue
            if det.confidence < self.min_confidence:
                continue
            world_xy = self.pixel_to_world(det.center_x, det.center_y)
            if world_xy is None:
                continue
            world_pose = self.make_world_pose(world_xy[0], world_xy[1], det.yaw_deg)
            target_pose = self.transform_pose(world_pose, self.target_frame)
            approach_pose = self.offset_pose_z(target_pose, self.approach_z_offset)
            candidates.append(
                {
                    "detection": det,
                    "world_pose": world_pose,
                    "target_pose": target_pose,
                    "approach_pose": approach_pose,
                }
            )

        candidates.sort(key=lambda item: item["detection"].confidence, reverse=True)
        self.publish_targets(msg.header, candidates)

    def pixel_to_world(self, u, v):
        x_min, x_max, y_min, y_max = self.view_bounds
        width = float(self.image_width)
        height = float(self.image_height)
        usable_w = width - 2.0 * self.view_margin_px
        usable_h = height - 2.0 * self.view_margin_px
        if usable_w <= 1.0 or usable_h <= 1.0:
            return None
        x = x_min + (float(u) - self.view_margin_px) * (x_max - x_min) / usable_w
        y = y_min + (height - self.view_margin_px - float(v)) * (y_max - y_min) / usable_h
        return x, y

    def make_world_pose(self, x, y, yaw_deg):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.world_frame
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = self.table_z + self.table_size_z * 0.5 + self.block_size_z + self.grasp_z_offset
        pose_stamped.pose.orientation = self.grasp_orientation(yaw_deg)
        return pose_stamped

    def grasp_orientation(self, yaw_deg):
        # The gripper approaches from above; yaw aligns the wrist with the detected block.
        yaw = math.radians(float(yaw_deg))
        quat = tft.quaternion_from_euler(math.pi, 0.0, yaw)
        pose = Pose()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose.orientation

    def transform_pose(self, pose_stamped, target_frame):
        if target_frame in ("", pose_stamped.header.frame_id):
            return pose_stamped
        try:
            self.tf_listener.waitForTransform(
                target_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            pose_stamped.header.stamp = rospy.Time(0)
            return self.tf_listener.transformPose(target_frame, pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Could not transform grasp pose from %s to %s: %s. Publishing world pose instead.",
                pose_stamped.header.frame_id,
                target_frame,
                exc,
            )
            return pose_stamped

    def offset_pose_z(self, pose_stamped, dz):
        approach = PoseStamped()
        approach.header = pose_stamped.header
        approach.pose.position.x = pose_stamped.pose.position.x
        approach.pose.position.y = pose_stamped.pose.position.y
        approach.pose.position.z = pose_stamped.pose.position.z + dz
        approach.pose.orientation = pose_stamped.pose.orientation
        return approach

    def publish_targets(self, source_header, candidates):
        pose_array = PoseArray()
        pose_array.header.stamp = source_header.stamp if source_header.stamp else rospy.Time.now()
        pose_array.header.frame_id = candidates[0]["target_pose"].header.frame_id if candidates else self.target_frame
        details = []

        for item in candidates:
            det = item["detection"]
            target_pose = item["target_pose"]
            world_pose = item["world_pose"]
            if self.publish_all:
                pose_array.poses.append(target_pose.pose)
            details.append(self.detail_item(det, world_pose, target_pose, item["approach_pose"]))

        self.poses_pub.publish(pose_array)
        if candidates:
            self.selected_pub.publish(candidates[0]["target_pose"])
            self.approach_pub.publish(candidates[0]["approach_pose"])

        msg = String()
        msg.data = json.dumps(
            {
                "source_topic": self.detections_topic,
                "target_frame": self.target_frame,
                "stable_only": self.stable_only,
                "detections": details,
            },
            sort_keys=True,
        )
        self.details_pub.publish(msg)

    def detail_item(self, det, world_pose, target_pose, approach_pose):
        return {
            "class_id": int(det.class_id),
            "class_name": str(det.class_name),
            "confidence": round(float(det.confidence), 4),
            "stable": bool(det.stable),
            "center_px": [int(det.center_x), int(det.center_y)],
            "bbox": [int(det.xmin), int(det.ymin), int(det.xmax), int(det.ymax)],
            "yaw_deg": round(float(det.yaw_deg), 3),
            "world_xyz": self.pose_xyz(world_pose.pose),
            "target_frame": str(target_pose.header.frame_id),
            "target_xyz": self.pose_xyz(target_pose.pose),
            "approach_xyz": self.pose_xyz(approach_pose.pose),
        }

    def pose_xyz(self, pose):
        return [
            round(float(pose.position.x), 4),
            round(float(pose.position.y), 4),
            round(float(pose.position.z), 4),
        ]


def main():
    GraspPoseConverter()
    rospy.spin()
    return 0


if __name__ == "__main__":
    main()
