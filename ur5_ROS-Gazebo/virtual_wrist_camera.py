#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import re
import sys
import time

import cv2
import cv_bridge
import numpy as np
import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


BLOCK_COLORS = {
    0: (46, 56, 204),
    1: (217, 122, 51),
    2: (242, 161, 61),
    3: (51, 184, 242),
    4: (71, 153, 61),
    5: (89, 178, 46),
    6: (204, 82, 199),
    7: (41, 122, 242),
    8: (102, 178, 51),
    9: (184, 82, 122),
}

FRUIT_COLORS = {
    "fruit_apple": (28, 38, 224),
    "fruit_pear": (52, 174, 135),
    "fruit_banana": (28, 205, 245),
}


class VirtualWristCamera(object):
    """Fallback image source for Docker/Mac Gazebo camera rendering issues."""

    SEGMENTS = {
        0: (1, 1, 1, 1, 1, 1, 0),
        1: (0, 1, 1, 0, 0, 0, 0),
        2: (1, 1, 0, 1, 1, 0, 1),
        3: (1, 1, 1, 1, 0, 0, 1),
        4: (0, 1, 1, 0, 0, 1, 1),
        5: (1, 0, 1, 1, 0, 1, 1),
        6: (1, 0, 1, 1, 1, 1, 1),
        7: (1, 1, 1, 0, 0, 0, 0),
        8: (1, 1, 1, 1, 1, 1, 1),
        9: (1, 1, 1, 1, 0, 1, 1),
    }

    def __init__(self):
        rospy.init_node("virtual_wrist_camera", anonymous=False)
        self.bridge = cv_bridge.CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/ur5/wrist_camera/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/ur5/wrist_camera/camera_info"
        )
        self.frame_id = rospy.get_param("~frame_id", "wrist_camera_optical_frame")
        self.width = int(rospy.get_param("~width", 1280))
        self.height = int(rospy.get_param("~height", 960))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.margin = float(rospy.get_param("~view_margin_px", 30.0))
        self.draw_all_digits = self.as_bool(rospy.get_param("~draw_all_digits", True))
        self.draw_target_grid = self.as_bool(rospy.get_param("~draw_target_grid", True))
        self.draw_fruits = self.as_bool(rospy.get_param("~draw_fruits", True))

        self.table_x = float(rospy.get_param("~table_x", 0.25))
        self.table_y = float(rospy.get_param("~table_y", 0.0))
        self.table_size_x = float(rospy.get_param("~table_size_x", 1.2))
        self.table_size_y = float(rospy.get_param("~table_size_y", 0.9))
        self.block_size_x = float(rospy.get_param("~block_size_x", 0.06))
        self.block_size_y = float(rospy.get_param("~block_size_y", 0.06))
        self.grid_cell_size = float(rospy.get_param("~grid_cell_size", 0.13))
        self.grid_origin_x = float(rospy.get_param("~grid_origin_x", -0.25))
        self.grid_origin_y = float(rospy.get_param("~grid_origin_y", -0.22))
        self.grid_spacing_x = float(rospy.get_param("~grid_spacing_x", 0.18))
        self.grid_spacing_y = float(rospy.get_param("~grid_spacing_y", 0.18))
        self.grid_cols = int(rospy.get_param("~grid_cols", 3))
        self.grid_rows = int(rospy.get_param("~grid_rows", 2))

        table_x_min = self.table_x - self.table_size_x * 0.5
        table_x_max = self.table_x + self.table_size_x * 0.5
        table_y_min = self.table_y - self.table_size_y * 0.5
        table_y_max = self.table_y + self.table_size_y * 0.5
        spawn_x_min = float(rospy.get_param("~spawn_x_min", 0.34))
        spawn_x_max = float(rospy.get_param("~spawn_x_max", 0.75))
        spawn_y_min = float(rospy.get_param("~spawn_y_min", -0.30))
        spawn_y_max = float(rospy.get_param("~spawn_y_max", 0.28))
        grid_x_min = self.grid_origin_x - self.grid_cell_size * 0.5
        grid_x_max = self.grid_origin_x + (self.grid_cols - 1) * self.grid_spacing_x + self.grid_cell_size * 0.5
        grid_y_min = self.grid_origin_y - self.grid_cell_size * 0.5
        grid_y_max = self.grid_origin_y + (self.grid_rows - 1) * self.grid_spacing_y + self.grid_cell_size * 0.5
        default_x_min = min(spawn_x_min, grid_x_min if self.draw_target_grid else spawn_x_min)
        default_x_max = max(spawn_x_max, grid_x_max if self.draw_target_grid else spawn_x_max)
        default_y_min = min(spawn_y_min, grid_y_min if self.draw_target_grid else spawn_y_min)
        default_y_max = max(spawn_y_max, grid_y_max if self.draw_target_grid else spawn_y_max)
        view_padding = float(rospy.get_param("~view_padding", 0.08))
        self.x_min = max(table_x_min, float(rospy.get_param("~view_x_min", default_x_min - view_padding)))
        self.x_max = min(table_x_max, float(rospy.get_param("~view_x_max", default_x_max + view_padding)))
        self.y_min = max(table_y_min, float(rospy.get_param("~view_y_min", default_y_min - view_padding)))
        self.y_max = min(table_y_max, float(rospy.get_param("~view_y_max", default_y_max + view_padding)))
        self.view_size_x = max(0.01, self.x_max - self.x_min)
        self.view_size_y = max(0.01, self.y_max - self.y_min)
        self.scale_x = (self.width - 2.0 * self.margin) / self.view_size_x
        self.scale_y = (self.height - 2.0 * self.margin) / self.view_size_y

        self.model_states = None
        self.last_model_state_warning = 0.0
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

        rospy.logwarn(
            "Using virtual wrist camera fallback on %s. Disable with "
            "virtual_wrist_camera:=false when the Gazebo camera publishes real frames.",
            self.image_topic,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def model_states_callback(self, msg):
        self.model_states = msg

    def iter_model_poses(self):
        if self.model_states is None:
            return []

        name_count = len(self.model_states.name)
        pose_count = len(self.model_states.pose)
        count = min(name_count, pose_count)
        if name_count != pose_count:
            now = time.time()
            if now - self.last_model_state_warning > 2.0:
                rospy.logwarn(
                    "Ignoring incomplete Gazebo ModelStates frame: names=%d poses=%d",
                    name_count,
                    pose_count,
                )
                self.last_model_state_warning = now
        return list(zip(self.model_states.name[:count], self.model_states.pose[:count]))

    def world_to_pixel(self, x, y):
        u = self.margin + (x - self.x_min) * self.scale_x
        v = self.height - self.margin - (y - self.y_min) * self.scale_y
        return float(u), float(v)

    def size_to_pixels(self, sx, sy):
        return max(1.0, sx * self.scale_x), max(1.0, sy * self.scale_y)

    def draw_table(self, image):
        x0, y0 = self.world_to_pixel(self.x_min, self.y_min)
        x1, y1 = self.world_to_pixel(self.x_max, self.y_max)
        top_left = (int(round(min(x0, x1))), int(round(min(y0, y1))))
        bottom_right = (int(round(max(x0, x1))), int(round(max(y0, y1))))
        cv2.rectangle(image, top_left, bottom_right, (88, 92, 91), -1)
        cv2.rectangle(image, top_left, bottom_right, (120, 125, 124), 2)

    def draw_grid(self, image, name, pose):
        match = re.match(r"target_grid_(\d+)$", name)
        if not match:
            return
        grid_id = int(match.group(1))
        u, v = self.world_to_pixel(pose.position.x, pose.position.y)
        w, h = self.size_to_pixels(self.grid_cell_size, self.grid_cell_size)
        x0 = int(round(u - w * 0.5))
        y0 = int(round(v - h * 0.5))
        x1 = int(round(u + w * 0.5))
        y1 = int(round(v + h * 0.5))
        cv2.rectangle(image, (x0, y0), (x1, y1), (220, 220, 198), -1)
        cv2.rectangle(image, (x0, y0), (x1, y1), (164, 74, 20), 2)
        self.draw_centered_text(image, str(grid_id), (u, v), 0.55, (35, 35, 35), 2)

    def draw_block(self, image, name, pose):
        match = re.match(r"digit_block_(\d+)$", name)
        if not match:
            return
        digit = int(match.group(1))
        if not self.draw_all_digits and digit not in (0, 1, 3):
            return
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = tft.euler_from_quaternion(quat)[2]
        u, v = self.world_to_pixel(pose.position.x, pose.position.y)
        w, h = self.size_to_pixels(self.block_size_x, self.block_size_y)
        rect = ((u, v), (w, h), -math.degrees(yaw))
        points = self.box_points(rect)
        color = BLOCK_COLORS.get(digit, (180, 180, 180))
        cv2.fillConvexPoly(image, points, color)
        cv2.polylines(image, [points], True, (245, 245, 245), 2)
        self.draw_digit_segments(image, digit, (u, v), w, h, yaw)

    def draw_fruit(self, image, name, pose):
        if name not in FRUIT_COLORS:
            return
        u, v = self.world_to_pixel(pose.position.x, pose.position.y)
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        yaw = tft.euler_from_quaternion(quat)[2]

        if name == "fruit_apple":
            radius = int(round(self.size_to_pixels(0.060, 0.060)[0] * 0.5))
            center = (int(round(u)), int(round(v)))
            cv2.circle(image, center, radius, FRUIT_COLORS[name], -1)
            cv2.circle(image, center, radius, (245, 245, 245), 2)
            cv2.circle(
                image,
                (center[0] - radius // 3, center[1] - radius // 3),
                max(2, radius // 5),
                (90, 105, 245),
                -1,
            )
            cv2.line(
                image,
                (center[0], center[1] - radius),
                (center[0] + radius // 4, center[1] - radius - radius // 2),
                (24, 58, 92),
                max(2, radius // 7),
                cv2.LINE_AA,
            )
            cv2.ellipse(
                image,
                (center[0] + radius // 2, center[1] - radius),
                (max(4, radius // 2), max(2, radius // 4)),
                -25,
                0,
                360,
                (35, 135, 42),
                -1,
            )
        elif name == "fruit_pear":
            radius = int(round(self.size_to_pixels(0.065, 0.065)[0] * 0.5))
            center = (int(round(u)), int(round(v)))
            cv2.circle(image, (center[0], center[1] + radius // 5), radius, FRUIT_COLORS[name], -1)
            cv2.circle(image, (center[0], center[1] - radius // 2), int(radius * 0.65), FRUIT_COLORS[name], -1)
            cv2.circle(image, (center[0], center[1] + radius // 5), radius, (245, 245, 245), 2)
            cv2.circle(image, (center[0], center[1] - radius // 2), int(radius * 0.65), (245, 245, 245), 2)
            cv2.line(
                image,
                (center[0], center[1] - radius),
                (center[0] + radius // 5, center[1] - radius - radius // 2),
                (22, 52, 86),
                max(2, radius // 8),
                cv2.LINE_AA,
            )
        elif name == "fruit_banana":
            length_px, width_px = self.size_to_pixels(0.090, 0.035)
            points = []
            for index in range(9):
                t = index / 8.0
                local_x = (t - 0.5) * length_px
                local_y = math.sin((t - 0.5) * math.pi) * width_px * 0.45
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                px = u + local_x * cos_yaw - local_y * sin_yaw
                py = v - (local_x * sin_yaw + local_y * cos_yaw)
                points.append((int(round(px)), int(round(py))))
            cv2.polylines(
                image,
                [np.array(points, dtype=np.int32)],
                False,
                FRUIT_COLORS[name],
                max(8, int(round(width_px))),
                cv2.LINE_AA,
            )
            cv2.polylines(
                image,
                [np.array(points, dtype=np.int32)],
                False,
                (250, 235, 90),
                max(4, int(round(width_px * 0.45))),
                cv2.LINE_AA,
            )
            for point in (points[0], points[-1]):
                cv2.circle(image, point, max(3, int(round(width_px * 0.25))), (30, 60, 95), -1)

    def draw_digit_segments(self, image, digit, center, block_w, block_h, yaw):
        patch_size = 96
        patch = np.zeros((patch_size, patch_size), dtype=np.uint8)
        scale = patch_size * 0.78
        length = scale * 0.58
        side_length = scale * 0.48
        width = max(3.0, scale * 0.10)
        top_y = patch_size * 0.5 - scale * 0.29
        mid_y = patch_size * 0.5
        bottom_y = patch_size * 0.5 + scale * 0.29
        left_x = patch_size * 0.5 - scale * 0.25
        right_x = patch_size * 0.5 + scale * 0.25

        def segment(cx, cy, sx, sy):
            x0 = int(round(cx - sx * 0.5))
            y0 = int(round(cy - sy * 0.5))
            x1 = int(round(cx + sx * 0.5))
            y1 = int(round(cy + sy * 0.5))
            cv2.rectangle(patch, (x0, y0), (x1, y1), 255, -1)

        segments = self.SEGMENTS.get(digit)
        if not segments:
            return
        if segments[0]:
            segment(patch_size * 0.5, top_y, length, width)
        if segments[1]:
            segment(right_x, (top_y + mid_y) * 0.5, width, side_length)
        if segments[2]:
            segment(right_x, (mid_y + bottom_y) * 0.5, width, side_length)
        if segments[3]:
            segment(patch_size * 0.5, bottom_y, length, width)
        if segments[4]:
            segment(left_x, (mid_y + bottom_y) * 0.5, width, side_length)
        if segments[5]:
            segment(left_x, (top_y + mid_y) * 0.5, width, side_length)
        if segments[6]:
            segment(patch_size * 0.5, mid_y, length, width)

        src = np.array(
            [
                [0.0, 0.0],
                [patch_size - 1.0, 0.0],
                [patch_size - 1.0, patch_size - 1.0],
                [0.0, patch_size - 1.0],
            ],
            dtype=np.float32,
        )
        dst = np.array(
            self.block_ordered_points(center, block_w * 0.70, block_h * 0.70, yaw),
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(
            patch,
            matrix,
            (image.shape[1], image.shape[0]),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        image[warped > 0] = (15, 15, 15)

    def block_ordered_points(self, center, width, height, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        half_w = width * 0.5
        half_h = height * 0.5
        corners = [(-half_w, half_h), (half_w, half_h), (half_w, -half_h), (-half_w, -half_h)]
        points = []
        for local_x, local_y in corners:
            world_x = local_x * cos_yaw - local_y * sin_yaw
            world_y = local_x * sin_yaw + local_y * cos_yaw
            points.append((center[0] + world_x, center[1] - world_y))
        return points

    def box_points(self, rect):
        if hasattr(cv2, "boxPoints"):
            points = cv2.boxPoints(rect)
        else:
            points = cv2.cv.BoxPoints(rect)
        return np.int0(points)

    def draw_centered_text(self, image, text, center, scale, color, thickness):
        font = cv2.FONT_HERSHEY_SIMPLEX
        size, _ = cv2.getTextSize(text, font, scale, thickness)
        x = int(round(center[0] - size[0] * 0.5))
        y = int(round(center[1] + size[1] * 0.5))
        cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)

    def render(self):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:, :] = (26, 29, 31)
        self.draw_table(image)
        if self.model_states is None:
            self.draw_centered_text(
                image,
                "waiting for Gazebo scene",
                (self.width * 0.5, self.height * 0.5),
                0.7,
                (220, 220, 220),
                2,
            )
            return image

        model_poses = self.iter_model_poses()
        if self.draw_target_grid:
            for name, pose in model_poses:
                self.draw_grid(image, name, pose)
        for name, pose in model_poses:
            self.draw_block(image, name, pose)
        if self.draw_fruits:
            for name, pose in model_poses:
                self.draw_fruit(image, name, pose)
        return image

    def make_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        fx = self.width / (2.0 * math.tan(1.3962634 * 0.5))
        fy = fx
        cx = self.width * 0.5
        cy = self.height * 0.5
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = "plumb_bob"
        return info

    def spin(self):
        delay = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.1
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            image = self.render()
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)
            self.info_pub.publish(self.make_camera_info(stamp))
            time.sleep(delay)


def main():
    camera = VirtualWristCamera()
    camera.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
