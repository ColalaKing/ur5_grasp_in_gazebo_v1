#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import random
import re
import sys

import cv2
import cv_bridge
import numpy as np
import rospy
import tf
import tf.transformations as tft
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class YoloDatasetCollector(object):
    """Collect wrist-camera images and YOLO labels for digit blocks."""

    def __init__(self):
        rospy.init_node("collect_yolo_dataset", anonymous=False)

        self.bridge = cv_bridge.CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/ur5/wrist_camera/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/ur5/wrist_camera/camera_info"
        )
        self.output_dir = rospy.get_param("~output_dir", "")
        self.classes = self.parse_classes(
            rospy.get_param("~classes", "0,1,2,3,4,5,6,7,8,9")
        )
        self.sample_count = int(rospy.get_param("~sample_count", 100))
        self.sample_period = float(rospy.get_param("~sample_period", 0.5))
        self.require_labels = self.as_bool(rospy.get_param("~require_labels", True))
        self.reset_train_list = self.as_bool(rospy.get_param("~reset_train_list", True))
        self.list_filename = rospy.get_param("~list_filename", "train.txt")
        self.start_index = int(rospy.get_param("~start_index", -1))
        self.min_box_px = float(rospy.get_param("~min_box_px", 8.0))
        self.label_mode = str(rospy.get_param("~label_mode", "topdown")).strip().lower()
        self.view_margin_px = float(rospy.get_param("~view_margin_px", 30.0))
        self.randomize_each_sample = self.as_bool(
            rospy.get_param("~randomize_each_sample", False)
        )
        self.random_settle_frames = max(
            0, int(rospy.get_param("~random_settle_frames", 2))
        )
        self.random_seed = int(rospy.get_param("~random_seed", rospy.get_param("~seed", 0)))

        self.block_size_x = float(rospy.get_param("~block_size_x", 0.06))
        self.block_size_y = float(rospy.get_param("~block_size_y", 0.06))
        self.block_size_z = float(rospy.get_param("~block_size_z", 0.04))
        self.block_spacing = float(rospy.get_param("~block_spacing", 0.09))
        self.spawn_x_min = float(rospy.get_param("~spawn_x_min", 0.34))
        self.spawn_x_max = float(rospy.get_param("~spawn_x_max", 0.75))
        self.spawn_y_min = float(rospy.get_param("~spawn_y_min", -0.30))
        self.spawn_y_max = float(rospy.get_param("~spawn_y_max", 0.28))
        self.table_z = float(rospy.get_param("~table_z", 0.10))
        self.table_x = float(rospy.get_param("~table_x", 0.25))
        self.table_y = float(rospy.get_param("~table_y", 0.0))
        self.table_size_x = float(rospy.get_param("~table_size_x", 1.2))
        self.table_size_y = float(rospy.get_param("~table_size_y", 0.9))
        self.table_size_z = float(rospy.get_param("~table_size_z", 0.20))
        self.camera_link_name = rospy.get_param(
            "~camera_link_name", "robot::wrist_camera_link"
        )
        self.camera_frame = rospy.get_param("~camera_frame", "wrist_camera_optical_frame")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.horizontal_fov = float(rospy.get_param("~horizontal_fov", 1.3962634))
        self.view_padding = float(rospy.get_param("~view_padding", 0.08))
        self.draw_target_grid = self.as_bool(rospy.get_param("~draw_target_grid", True))

        if not self.output_dir:
            raise RuntimeError("output_dir is required")
        self.images_dir = os.path.join(self.output_dir, "images")
        self.labels_dir = os.path.join(self.output_dir, "labels")
        self.darknet_root = os.path.dirname(os.path.abspath(self.output_dir))
        self.ensure_dir(self.images_dir)
        self.ensure_dir(self.labels_dir)
        self.list_path = os.path.join(self.output_dir, self.list_filename)
        if self.reset_train_list:
            open(self.list_path, "w").close()
            self.sample_index = self.start_index if self.start_index >= 0 else 0
        else:
            self.sample_index = self.start_index if self.start_index >= 0 else self.next_sample_index()

        self.camera_info = None
        self.model_states = None
        self.camera_link_pose = None
        self.last_save = rospy.Time(0)
        self.saved_count = 0
        self.tf_listener = tf.TransformListener()
        self.class_to_index = dict((digit, index) for index, digit in enumerate(self.classes))
        self.rng = random.Random(self.random_seed if self.random_seed != 0 else None)
        self.randomized_for_sample = False
        self.pending_random_settle_frames = 0
        self.set_model_state = None
        if self.randomize_each_sample:
            try:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=8.0)
                self.set_model_state = rospy.ServiceProxy(
                    "/gazebo/set_model_state", SetModelState
                )
            except rospy.ROSException as exc:
                raise RuntimeError(
                    "randomize_each_sample requires /gazebo/set_model_state: %s" % exc
                )
        self.link_to_optical = tft.euler_matrix(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        self.configure_topdown_view()
        self.view_size_x = max(0.01, self.view_x_max - self.view_x_min)
        self.view_size_y = max(0.01, self.view_y_max - self.view_y_min)

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

        rospy.loginfo(
            "Collecting YOLO dataset: output_dir=%s list=%s classes=%s sample_count=%d randomize_each_sample=%s",
            self.output_dir,
            self.list_path,
            self.classes,
            self.sample_count,
            self.randomize_each_sample,
        )

    def parse_classes(self, value):
        raw = value if isinstance(value, list) else str(value).replace(";", ",").split(",")
        classes = []
        for item in raw:
            token = str(item).strip()
            if not token:
                continue
            digit = int(token)
            if digit not in classes:
                classes.append(digit)
        return classes or list(range(10))

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def ensure_dir(self, path):
        if not os.path.isdir(path):
            os.makedirs(path)

    def configure_topdown_view(self):
        table_x_min = self.table_x - self.table_size_x * 0.5
        table_x_max = self.table_x + self.table_size_x * 0.5
        table_y_min = self.table_y - self.table_size_y * 0.5
        table_y_max = self.table_y + self.table_size_y * 0.5
        grid_cell_size = float(rospy.get_param("~grid_cell_size", 0.13))
        grid_origin_x = float(rospy.get_param("~grid_origin_x", -0.25))
        grid_origin_y = float(rospy.get_param("~grid_origin_y", -0.22))
        grid_spacing_x = float(rospy.get_param("~grid_spacing_x", 0.18))
        grid_spacing_y = float(rospy.get_param("~grid_spacing_y", 0.18))
        grid_cols = int(rospy.get_param("~grid_cols", 3))
        grid_rows = int(rospy.get_param("~grid_rows", 2))

        grid_x_min = grid_origin_x - grid_cell_size * 0.5
        grid_x_max = grid_origin_x + (grid_cols - 1) * grid_spacing_x + grid_cell_size * 0.5
        grid_y_min = grid_origin_y - grid_cell_size * 0.5
        grid_y_max = grid_origin_y + (grid_rows - 1) * grid_spacing_y + grid_cell_size * 0.5
        default_x_min = min(self.spawn_x_min, grid_x_min if self.draw_target_grid else self.spawn_x_min)
        default_x_max = max(self.spawn_x_max, grid_x_max if self.draw_target_grid else self.spawn_x_max)
        default_y_min = min(self.spawn_y_min, grid_y_min if self.draw_target_grid else self.spawn_y_min)
        default_y_max = max(self.spawn_y_max, grid_y_max if self.draw_target_grid else self.spawn_y_max)

        self.view_x_min = max(
            table_x_min,
            float(rospy.get_param("~view_x_min", default_x_min - self.view_padding)),
        )
        self.view_x_max = min(
            table_x_max,
            float(rospy.get_param("~view_x_max", default_x_max + self.view_padding)),
        )
        self.view_y_min = max(
            table_y_min,
            float(rospy.get_param("~view_y_min", default_y_min - self.view_padding)),
        )
        self.view_y_max = min(
            table_y_max,
            float(rospy.get_param("~view_y_max", default_y_max + self.view_padding)),
        )

    def next_sample_index(self):
        highest = -1
        pattern = re.compile(r"^frame_(\d+)\.jpg$")
        for name in os.listdir(self.images_dir):
            match = pattern.match(name)
            if match:
                highest = max(highest, int(match.group(1)))
        return highest + 1

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def model_states_callback(self, msg):
        self.model_states = msg

    def link_states_callback(self, msg):
        if self.camera_link_name in msg.name:
            self.camera_link_pose = msg.pose[msg.name.index(self.camera_link_name)]

    def image_callback(self, msg):
        if self.saved_count >= self.sample_count:
            return
        if self.model_states is None:
            return
        if self.label_mode != "topdown" and self.camera_info is None:
            return
        now = rospy.Time.now()
        if (now - self.last_save).to_sec() < self.sample_period:
            return

        if self.randomize_each_sample:
            if self.pending_random_settle_frames > 0:
                self.pending_random_settle_frames -= 1
                return
            if not self.randomized_for_sample:
                if self.randomize_block_models():
                    self.randomized_for_sample = True
                    self.pending_random_settle_frames = self.random_settle_frames
                return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as exc:
            rospy.logerr("Failed to convert image: %s", exc)
            return

        if self.label_mode == "topdown":
            labels = self.build_topdown_labels(image.shape[1], image.shape[0])
        else:
            world_to_camera = self.get_world_to_camera_matrix()
            if world_to_camera is None:
                rospy.logwarn_throttle(2.0, "Waiting for wrist camera pose")
                return
            labels = self.build_labels(world_to_camera, image.shape[1], image.shape[0])
        if self.require_labels and not labels:
            rospy.logwarn_throttle(2.0, "No visible target labels in current frame")
            self.randomized_for_sample = False
            return

        stem = "frame_%06d" % self.sample_index
        image_path = os.path.join(self.images_dir, stem + ".jpg")
        label_path = os.path.join(self.labels_dir, stem + ".txt")
        darknet_label_path = os.path.join(self.images_dir, stem + ".txt")
        cv2.imwrite(image_path, image)
        with open(label_path, "w") as label_file:
            for label in labels:
                label_file.write(label + "\n")
        with open(darknet_label_path, "w") as label_file:
            for label in labels:
                label_file.write(label + "\n")
        with open(self.list_path, "a") as list_file:
            list_file.write(os.path.relpath(image_path, self.darknet_root) + "\n")

        self.saved_count += 1
        self.sample_index += 1
        self.last_save = now
        self.randomized_for_sample = False
        rospy.loginfo("Saved YOLO sample %d/%d: %s", self.saved_count, self.sample_count, stem)
        if self.saved_count >= self.sample_count:
            rospy.loginfo("Dataset collection complete: %s", self.output_dir)
            rospy.signal_shutdown("dataset collection complete")

    def randomize_block_models(self):
        if self.set_model_state is None:
            return False
        poses = self.generate_random_block_poses()
        for digit, pose_2d in enumerate(poses):
            name = "digit_block_%d" % digit
            if self.model_states is not None and name not in self.model_states.name:
                continue
            state = ModelState()
            state.model_name = name
            state.pose = self.make_pose(pose_2d[0], pose_2d[1], pose_2d[2], pose_2d[3])
            state.twist = Twist()
            state.reference_frame = "world"
            try:
                response = self.set_model_state(state)
            except rospy.ServiceException as exc:
                rospy.logerr("Failed to randomize %s: %s", name, exc)
                return False
            if not response.success:
                rospy.logerr("Failed to randomize %s: %s", name, response.status_message)
                return False
        return True

    def generate_random_block_poses(self):
        poses = []
        z = self.table_z + self.table_size_z * 0.5 + 0.002
        for index in range(10):
            accepted = False
            x = self.spawn_x_min + (index % 5) * self.block_spacing
            y = self.spawn_y_min + (index / 5) * self.block_spacing
            yaw = 0.0
            for _ in range(200):
                candidate_x = self.rng.uniform(self.spawn_x_min, self.spawn_x_max)
                candidate_y = self.rng.uniform(self.spawn_y_min, self.spawn_y_max)
                candidate_yaw = self.rng.uniform(-math.pi, math.pi)
                if self.far_enough(candidate_x, candidate_y, poses):
                    x = candidate_x
                    y = candidate_y
                    yaw = candidate_yaw
                    accepted = True
                    break
            if not accepted:
                rospy.logwarn_throttle(
                    3.0, "Falling back to grid pose while randomizing block %d", index
                )
            poses.append((x, y, z, yaw))
        return poses

    def far_enough(self, x, y, poses):
        for pose in poses:
            dx = x - pose[0]
            dy = y - pose[1]
            if math.sqrt(dx * dx + dy * dy) < self.block_spacing:
                return False
        return True

    def make_pose(self, x, y, z, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def get_world_to_camera_matrix(self):
        if self.camera_link_pose is not None:
            world_to_link = self.pose_to_matrix(self.camera_link_pose)
            world_to_optical = np.dot(world_to_link, self.link_to_optical)
            return np.linalg.inv(world_to_optical)
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.camera_frame, self.world_frame, rospy.Time(0)
            )
            return np.dot(tft.translation_matrix(trans), tft.quaternion_matrix(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def pose_to_matrix(self, pose):
        translation = tft.translation_matrix(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        rotation = tft.quaternion_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        return np.dot(translation, rotation)

    def build_labels(self, world_to_camera, width, height):
        labels = []
        fx, fy, cx, cy = self.camera_intrinsics(width, height)
        for digit in self.classes:
            name = "digit_block_%d" % digit
            if name not in self.model_states.name:
                continue
            pose = self.model_states.pose[self.model_states.name.index(name)]
            pixels = self.project_block(pose, world_to_camera, fx, fy, cx, cy)
            if len(pixels) < 2:
                continue
            xs = [pixel[0] for pixel in pixels]
            ys = [pixel[1] for pixel in pixels]
            xmin = max(0.0, min(xs))
            xmax = min(float(width - 1), max(xs))
            ymin = max(0.0, min(ys))
            ymax = min(float(height - 1), max(ys))
            if xmax - xmin < self.min_box_px or ymax - ymin < self.min_box_px:
                continue
            x_center = ((xmin + xmax) * 0.5) / float(width)
            y_center = ((ymin + ymax) * 0.5) / float(height)
            box_w = (xmax - xmin) / float(width)
            box_h = (ymax - ymin) / float(height)
            labels.append(
                "%d %.6f %.6f %.6f %.6f"
                % (self.class_to_index[digit], x_center, y_center, box_w, box_h)
            )
        return labels

    def build_topdown_labels(self, width, height):
        labels = []
        for digit in self.classes:
            name = "digit_block_%d" % digit
            if name not in self.model_states.name:
                continue
            pose = self.model_states.pose[self.model_states.name.index(name)]
            pixels = self.topdown_block_pixels(pose, width, height)
            xs = [pixel[0] for pixel in pixels]
            ys = [pixel[1] for pixel in pixels]
            xmin = max(0.0, min(xs))
            xmax = min(float(width - 1), max(xs))
            ymin = max(0.0, min(ys))
            ymax = min(float(height - 1), max(ys))
            if xmax - xmin < self.min_box_px or ymax - ymin < self.min_box_px:
                continue
            x_center = ((xmin + xmax) * 0.5) / float(width)
            y_center = ((ymin + ymax) * 0.5) / float(height)
            box_w = (xmax - xmin) / float(width)
            box_h = (ymax - ymin) / float(height)
            labels.append(
                "%d %.6f %.6f %.6f %.6f"
                % (self.class_to_index[digit], x_center, y_center, box_w, box_h)
            )
        return labels

    def topdown_block_pixels(self, pose, width, height):
        yaw = tft.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )[2]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        hx = self.block_size_x * 0.5
        hy = self.block_size_y * 0.5
        pixels = []
        for local_x, local_y in ((-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)):
            world_x = pose.position.x + local_x * cos_yaw - local_y * sin_yaw
            world_y = pose.position.y + local_x * sin_yaw + local_y * cos_yaw
            pixels.append(self.topdown_world_to_pixel(world_x, world_y, width, height))
        return pixels

    def topdown_world_to_pixel(self, x, y, width, height):
        usable_w = float(width) - 2.0 * self.view_margin_px
        usable_h = float(height) - 2.0 * self.view_margin_px
        u = self.view_margin_px + (x - self.view_x_min) * usable_w / self.view_size_x
        v = (
            float(height)
            - self.view_margin_px
            - (y - self.view_y_min) * usable_h / self.view_size_y
        )
        return float(u), float(v)

    def camera_intrinsics(self, width, height):
        k = self.camera_info.K
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx <= 0.0:
            fx = width / (2.0 * math.tan(self.horizontal_fov * 0.5))
        if fy <= 0.0:
            fy = fx
        if cx <= 0.0:
            cx = width * 0.5
        if cy <= 0.0:
            cy = height * 0.5
        return fx, fy, cx, cy

    def project_block(self, pose, world_to_camera, fx, fy, cx, cy):
        model_to_world = self.pose_to_matrix(pose)
        pixels = []
        for point in self.block_corners():
            point_world = np.dot(model_to_world, point)
            point_camera = np.dot(world_to_camera, point_world)
            z = point_camera[2]
            if z <= 0.02:
                continue
            u = fx * (point_camera[0] / z) + cx
            v = fy * (point_camera[1] / z) + cy
            if np.isfinite(u) and np.isfinite(v):
                pixels.append((float(u), float(v)))
        return pixels

    def block_corners(self):
        hx = self.block_size_x * 0.5
        hy = self.block_size_y * 0.5
        points = []
        for x in (-hx, hx):
            for y in (-hy, hy):
                for z in (0.0, self.block_size_z):
                    points.append(np.array([x, y, z, 1.0], dtype=np.float64))
        return points


def main():
    try:
        YoloDatasetCollector()
    except Exception as exc:
        rospy.logfatal("Cannot start YOLO dataset collector: %s", exc)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
