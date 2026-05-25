#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import re
import sys
import time

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ur5_notebook.msg import Detection2D
from ur5_notebook.msg import Detection2DArray


class TraditionalDigitDetector(object):
    """Color, contour, template, and hybrid baselines for digit-block detection."""

    DIGIT_RGB = {
        0: (0.80, 0.22, 0.18),
        1: (0.20, 0.48, 0.85),
        2: (0.18, 0.63, 0.95),
        3: (0.95, 0.72, 0.20),
        4: (0.24, 0.60, 0.28),
        5: (0.18, 0.70, 0.35),
        6: (0.78, 0.32, 0.80),
        7: (0.95, 0.48, 0.16),
        8: (0.20, 0.70, 0.40),
        9: (0.48, 0.32, 0.72),
    }

    COLOR_NAMES = {
        0: "red",
        1: "blue",
        2: "cyan",
        3: "yellow",
        4: "green",
        5: "green",
        6: "magenta",
        7: "orange",
        8: "green",
        9: "purple",
    }

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
        rospy.init_node("traditional_detection", anonymous=False)

        self.bridge = cv_bridge.CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/ur5/wrist_camera/image_raw")
        self.method = rospy.get_param("~method", "digit").lower()
        self.target_classes = self.parse_classes(
            rospy.get_param("~classes", "0,1,2,3,4,5,6,7,8,9")
        )
        self.confidence_threshold = float(rospy.get_param("~confidence_threshold", 0.40))
        self.confirm_frames = max(1, int(rospy.get_param("~confirm_frames", 3)))
        self.confirm_distance_px = float(rospy.get_param("~confirm_distance_px", 35.0))
        self.min_area = float(rospy.get_param("~min_area", 350.0))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.35))
        self.min_saturation = int(rospy.get_param("~min_saturation", 45))
        self.min_value = int(rospy.get_param("~min_value", 35))
        self.dark_threshold = int(rospy.get_param("~dark_threshold", 70))
        self.color_distance_threshold = float(rospy.get_param("~color_distance_threshold", 0.36))
        self.template_threshold = float(rospy.get_param("~template_threshold", 0.18))
        self.color_assist_classes = self.parse_optional_classes(
            rospy.get_param("~color_assist_classes", "6,9")
        )
        self.color_assist_confidence = float(rospy.get_param("~color_assist_confidence", 0.45))
        self.use_detection_roi = self.as_bool(rospy.get_param("~use_detection_roi", True))
        self.roi_x_min_ratio = float(rospy.get_param("~roi_x_min_ratio", 0.50))
        self.roi_x_max_ratio = float(rospy.get_param("~roi_x_max_ratio", 1.00))
        self.roi_y_min_ratio = float(rospy.get_param("~roi_y_min_ratio", 0.00))
        self.roi_y_max_ratio = float(rospy.get_param("~roi_y_max_ratio", 1.00))
        self.min_extent = float(rospy.get_param("~min_extent", 0.35))
        self.max_candidate_width_ratio = float(
            rospy.get_param("~max_candidate_width_ratio", 0.18)
        )
        self.max_candidate_height_ratio = float(
            rospy.get_param("~max_candidate_height_ratio", 0.24)
        )
        self.show_window = self.as_bool(rospy.get_param("~show_window", False))
        self.publish_annotated = self.as_bool(rospy.get_param("~publish_annotated", True))
        self.window_name = rospy.get_param("~window_name", "traditional_detection")
        self.results_topic = rospy.get_param("~results_topic", "/traditional_detection/results")
        self.annotated_topic = rospy.get_param(
            "~annotated_topic", "/traditional_detection/image_annotated"
        )
        self.stats_topic = rospy.get_param("~stats_topic", "/traditional_detection/stats")
        self.details_topic = rospy.get_param("~details_topic", "/traditional_detection/details")

        self.view_margin_px = float(rospy.get_param("~view_margin_px", 30.0))
        self.view_bounds = self.build_virtual_view_bounds()

        self.templates = self.build_digit_templates(80)
        self.color_vectors = self.build_color_vectors()
        self.tracks = {}
        self.frame_index = 0
        self.window_ready = False
        self.window_failed = False

        self.results_pub = rospy.Publisher(self.results_topic, Detection2DArray, queue_size=1)
        self.annotated_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)
        self.stats_pub = rospy.Publisher(self.stats_topic, String, queue_size=1)
        self.details_pub = rospy.Publisher(self.details_topic, String, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )

        rospy.loginfo(
            "traditional_detection started: method=%s classes=%s image_topic=%s",
            self.method,
            self.target_classes,
            self.image_topic,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def parse_classes(self, value):
        raw_items = value if isinstance(value, list) else str(value).replace(";", ",").split(",")
        classes = []
        for item in raw_items:
            token = str(item).strip()
            if not token:
                continue
            try:
                digit = int(token)
            except ValueError:
                rospy.logwarn("Ignoring invalid class id '%s'", token)
                continue
            if 0 <= digit <= 9 and digit not in classes:
                classes.append(digit)
        return classes or list(range(10))

    def parse_optional_classes(self, value):
        raw_items = value if isinstance(value, list) else str(value).replace(";", ",").split(",")
        classes = []
        for item in raw_items:
            token = str(item).strip()
            if not token:
                continue
            try:
                digit = int(token)
            except ValueError:
                rospy.logwarn("Ignoring invalid assist class id '%s'", token)
                continue
            if 0 <= digit <= 9 and digit not in classes:
                classes.append(digit)
        return classes

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

    def build_color_vectors(self):
        vectors = {}
        for digit, rgb in self.DIGIT_RGB.items():
            arr = np.array(rgb, dtype=np.float32)
            vectors[digit] = arr / (np.sum(arr) + 1e-6)
        return vectors

    def build_digit_templates(self, size):
        templates = {}
        for digit in range(10):
            base = self.render_digit_template(digit, size)
            rotated = []
            for angle in range(0, 360, 10):
                matrix = cv2.getRotationMatrix2D((size / 2.0, size / 2.0), angle, 1.0)
                image = cv2.warpAffine(
                    base, matrix, (size, size), flags=cv2.INTER_NEAREST, borderValue=0
                )
                normalized = self.normalize_digit_mask(image, size)
                rotated.append((float(angle), normalized if normalized is not None else image))
            templates[digit] = rotated
        return templates

    def render_digit_template(self, digit, size):
        image = np.zeros((size, size), dtype=np.uint8)
        scale = size * 0.78
        length = scale * 0.58
        side_length = scale * 0.48
        width = max(3.0, scale * 0.10)
        top_y = size * 0.5 - scale * 0.29
        mid_y = size * 0.5
        bottom_y = size * 0.5 + scale * 0.29
        left_x = size * 0.5 - scale * 0.25
        right_x = size * 0.5 + scale * 0.25

        def segment(cx, cy, sx, sy):
            x0 = int(round(cx - sx * 0.5))
            y0 = int(round(cy - sy * 0.5))
            x1 = int(round(cx + sx * 0.5))
            y1 = int(round(cy + sy * 0.5))
            cv2.rectangle(image, (x0, y0), (x1, y1), 255, -1)

        segments = self.SEGMENTS[digit]
        if segments[0]:
            segment(size * 0.5, top_y, length, width)
        if segments[1]:
            segment(right_x, (top_y + mid_y) * 0.5, width, side_length)
        if segments[2]:
            segment(right_x, (mid_y + bottom_y) * 0.5, width, side_length)
        if segments[3]:
            segment(size * 0.5, bottom_y, length, width)
        if segments[4]:
            segment(left_x, (mid_y + bottom_y) * 0.5, width, side_length)
        if segments[5]:
            segment(left_x, (top_y + mid_y) * 0.5, width, side_length)
        if segments[6]:
            segment(size * 0.5, mid_y, length, width)
        return image

    def image_callback(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as exc:
            rospy.logerr("Failed to convert camera image: %s", exc)
            return

        started_at = time.time()
        detections = self.detect(image)
        detections = self.nms(detections, 0.30)
        detections = [det for det in detections if det["confidence"] >= self.confidence_threshold]
        for det in detections:
            self.add_world_center(det, image.shape)
        self.update_stability(detections)
        elapsed_ms = (time.time() - started_at) * 1000.0

        self.publish_results(image_msg.header, detections)
        self.publish_stats(image_msg.header, elapsed_ms, detections)
        self.publish_details(image_msg.header, detections)

        annotated = self.draw_detections(image.copy(), detections)
        if self.publish_annotated:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header = image_msg.header
                self.annotated_pub.publish(annotated_msg)
            except cv_bridge.CvBridgeError as exc:
                rospy.logerr("Failed to publish annotated image: %s", exc)
        self.show_image(annotated)

    def detect(self, image):
        if self.method == "color":
            detections = self.detect_by_color(image)
        else:
            detections = self.detect_candidates_then_classify(image, self.method)
        return detections

    def detect_by_color(self, image):
        image_float = image.astype(np.float32) / 255.0
        bgr_sum = np.sum(image_float, axis=2) + 1e-6
        rgb_norm = np.dstack(
            (
                image_float[:, :, 2] / bgr_sum,
                image_float[:, :, 1] / bgr_sum,
                image_float[:, :, 0] / bgr_sum,
            )
        )
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        valid = (hsv[:, :, 1] >= self.min_saturation) & (hsv[:, :, 2] >= self.min_value)
        detections = []
        image_area = float(image.shape[0] * image.shape[1])
        kernel = np.ones((3, 3), dtype=np.uint8)
        for digit in self.target_classes:
            target = self.color_vectors[digit]
            distance = np.linalg.norm(rgb_norm - target.reshape((1, 1, 3)), axis=2)
            mask = np.zeros(distance.shape, dtype=np.uint8)
            mask[(distance <= self.color_distance_threshold) & valid] = 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            for contour in self.find_contours(mask):
                det = self.contour_to_detection(contour, image.shape, image_area)
                if det is None:
                    continue
                roi_distance = distance[det["ymin"] : det["ymax"], det["xmin"] : det["xmax"]]
                confidence = 1.0 - min(1.0, float(np.mean(roi_distance)) / self.color_distance_threshold)
                det.update(
                    {
                        "class_id": digit,
                        "class_name": str(digit),
                        "confidence": max(0.0, min(1.0, confidence)),
                        "yaw_deg": float(det.get("contour_yaw_deg", 0.0)),
                        "yaw_source": "contour",
                        "stable": False,
                    }
                )
                detections.append(det)
        return detections

    def detect_candidates_then_classify(self, image, method):
        mask = self.block_candidate_mask(image)
        image_area = float(image.shape[0] * image.shape[1])
        detections = []
        for contour in self.find_contours(mask):
            det = self.contour_to_detection(contour, image.shape, image_area)
            if det is None:
                continue
            roi = image[det["ymin"] : det["ymax"], det["xmin"] : det["xmax"]]
            aligned_roi = self.extract_rotated_roi(image, contour)
            classification = self.classify_roi(
                aligned_roi if aligned_roi is not None else roi, method
            )
            if classification is None:
                continue
            digit, confidence = classification[:2]
            color_digit, color_confidence = self.classify_by_color(roi)
            digit, confidence, color_corrected = self.apply_color_assist(
                digit, confidence, color_digit, color_confidence
            )
            if digit not in self.target_classes:
                continue
            yaw_deg, yaw_source = self.estimate_yaw(roi, contour, digit)
            det.update(
                {
                    "class_id": digit,
                    "class_name": str(digit),
                    "confidence": max(0.0, min(1.0, confidence)),
                    "color_digit": -1 if color_digit is None else int(color_digit),
                    "color_name": self.color_name(color_digit),
                    "color_confidence": max(0.0, min(1.0, color_confidence)),
                    "color_corrected": bool(color_corrected),
                    "yaw_deg": float(yaw_deg),
                    "yaw_source": str(yaw_source),
                    "stable": False,
                }
            )
            detections.append(det)
        return detections

    def block_candidate_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        mask[(hsv[:, :, 1] >= self.min_saturation) & (hsv[:, :, 2] >= self.min_value)] = 255
        if self.use_detection_roi:
            roi_mask = np.zeros(mask.shape, dtype=np.uint8)
            height, width = mask.shape[:2]
            x0 = int(round(max(0.0, min(1.0, self.roi_x_min_ratio)) * width))
            x1 = int(round(max(0.0, min(1.0, self.roi_x_max_ratio)) * width))
            y0 = int(round(max(0.0, min(1.0, self.roi_y_min_ratio)) * height))
            y1 = int(round(max(0.0, min(1.0, self.roi_y_max_ratio)) * height))
            roi_mask[y0:y1, x0:x1] = 255
            mask = cv2.bitwise_and(mask, roi_mask)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        return mask

    def contour_to_detection(self, contour, shape, image_area):
        area = cv2.contourArea(contour)
        if area < self.min_area or area > image_area * self.max_area_ratio:
            return None
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 3 or h <= 3:
            return None
        if w > shape[1] * self.max_candidate_width_ratio:
            return None
        if h > shape[0] * self.max_candidate_height_ratio:
            return None
        extent = area / float(max(1, w * h))
        if extent < self.min_extent:
            return None
        return {
            "xmin": int(max(0, x)),
            "ymin": int(max(0, y)),
            "xmax": int(min(shape[1] - 1, x + w)),
            "ymax": int(min(shape[0] - 1, y + h)),
            "center_x": int(x + w * 0.5),
            "center_y": int(y + h * 0.5),
            "contour_yaw_deg": self.contour_yaw_deg(contour),
        }

    def contour_yaw_deg(self, contour):
        rect = cv2.minAreaRect(contour)
        points = self.order_box_points(self.box_points(rect).astype(np.float32))
        dx = float(points[1][0] - points[0][0])
        dy = float(points[1][1] - points[0][1])
        image_angle = math.degrees(math.atan2(dy, dx))
        return self.normalize_degrees(-image_angle)

    def classify_roi(self, roi, method):
        if roi.size == 0:
            return None
        if method in ("digit", "digits", "number", "numbers"):
            return self.classify_by_digit_template(roi)
        if method == "contour":
            return self.classify_by_segments(roi)
        if method == "template":
            return self.classify_by_template(roi)
        if method == "hybrid":
            color_digit, color_conf = self.classify_by_color(roi)
            template_result = self.classify_by_template(roi)
            contour_result = self.classify_by_segments(roi)
            candidates = []
            if color_digit is not None:
                candidates.append((color_digit, color_conf * 0.55))
            if template_result is not None:
                candidates.append((template_result[0], template_result[1] * 0.30))
            if contour_result is not None:
                candidates.append((contour_result[0], contour_result[1] * 0.15))
            if not candidates:
                return None
            scores = {}
            for digit, score in candidates:
                scores[digit] = scores.get(digit, 0.0) + score
            digit = max(scores, key=scores.get)
            return digit, scores[digit]
        return self.classify_by_color(roi)

    def extract_rotated_roi(self, image, contour):
        rect = cv2.minAreaRect(contour)
        (center_x, center_y), (rect_w, rect_h), _ = rect
        if rect_w < 4.0 or rect_h < 4.0:
            return None
        points = self.box_points(rect).astype(np.float32)
        points = self.order_box_points(points)
        side = int(round(max(rect_w, rect_h) * 1.35))
        side = max(32, min(160, side))
        dst = np.array(
            [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(points, dst)
        try:
            return cv2.warpPerspective(
                image,
                matrix,
                (side, side),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
        except cv2.error:
            return None

    def box_points(self, rect):
        if hasattr(cv2, "boxPoints"):
            points = cv2.boxPoints(rect)
        else:
            points = cv2.cv.BoxPoints(rect)
        return np.array(points)

    def order_box_points(self, points):
        ordered = np.zeros((4, 2), dtype=np.float32)
        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1).reshape(-1)
        ordered[0] = points[np.argmin(sums)]
        ordered[2] = points[np.argmax(sums)]
        ordered[1] = points[np.argmin(diffs)]
        ordered[3] = points[np.argmax(diffs)]
        return ordered

    def classify_by_color(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = (hsv[:, :, 1] >= self.min_saturation) & (hsv[:, :, 2] >= self.min_value)
        if np.count_nonzero(mask) < 10:
            return None, 0.0
        bgr = roi.astype(np.float32) / 255.0
        rgb = np.dstack((bgr[:, :, 2], bgr[:, :, 1], bgr[:, :, 0]))
        mean_rgb = np.mean(rgb[mask], axis=0)
        mean_rgb = mean_rgb / (np.sum(mean_rgb) + 1e-6)
        best_digit = None
        best_distance = 999.0
        for digit in range(10):
            distance = np.linalg.norm(mean_rgb - self.color_vectors[digit])
            if distance < best_distance:
                best_digit = digit
                best_distance = distance
        confidence = 1.0 - min(1.0, best_distance / self.color_distance_threshold)
        return best_digit, confidence

    def apply_color_assist(self, digit, confidence, color_digit, color_confidence):
        if color_digit is None:
            return digit, confidence, False
        if color_digit == digit:
            return digit, max(confidence, confidence * 0.75 + color_confidence * 0.25), False
        if (
            digit in self.color_assist_classes
            and color_digit in self.color_assist_classes
            and color_confidence >= self.color_assist_confidence
        ):
            assisted_confidence = max(confidence * 0.85, color_confidence * 0.85)
            return color_digit, assisted_confidence, True
        return digit, confidence, False

    def color_name(self, digit):
        if digit is None:
            return "unknown"
        return self.COLOR_NAMES.get(int(digit), "unknown")

    def classify_by_template(self, roi):
        dark = self.dark_digit_mask(roi)
        if np.count_nonzero(dark) < 8:
            return None
        resized = self.normalize_digit_mask(dark, 80)
        if resized is None:
            return None
        best_digit = None
        best_score = 0.0
        for digit in range(10):
            for _, templ in self.templates[digit]:
                score = self.iou(resized, templ)
                if score > best_score:
                    best_score = score
                    best_digit = digit
        if best_score < self.template_threshold:
            return None
        return best_digit, best_score

    def classify_by_digit_template(self, roi):
        template_result = self.classify_by_template(roi)
        if template_result is None:
            return None
        digit, template_confidence = template_result
        segment_result = self.classify_by_segments(roi)
        if segment_result is not None and segment_result[0] == digit:
            confidence = min(1.0, template_confidence * 0.75 + segment_result[1] * 0.25)
        else:
            confidence = template_confidence
        return digit, confidence

    def estimate_yaw(self, roi, contour, digit):
        digit_yaw = self.estimate_yaw_by_digit_template(roi, digit)
        if digit_yaw is not None:
            return digit_yaw, "digit_template"
        return self.contour_yaw_deg(contour), "contour"

    def estimate_yaw_by_digit_template(self, roi, digit):
        dark = self.dark_digit_mask(roi)
        if np.count_nonzero(dark) < 8:
            return None
        resized = self.normalize_digit_mask(dark, 80)
        if resized is None:
            return None
        best_score = 0.0
        best_angle = None
        for angle, templ in self.templates.get(int(digit), []):
            score = self.iou(resized, templ)
            if score > best_score:
                best_score = score
                best_angle = angle
        if best_angle is None or best_score < self.template_threshold:
            return None
        return self.normalize_degrees(-float(best_angle))

    def normalize_degrees(self, value):
        normalized = (float(value) + 180.0) % 360.0 - 180.0
        if normalized == -180.0:
            return 180.0
        return normalized

    def classify_by_segments(self, roi):
        dark = self.dark_digit_mask(roi)
        if np.count_nonzero(dark) < 8:
            return None
        resized = self.normalize_digit_mask(dark, 80)
        if resized is None:
            return None

        boxes = [
            (24, 13, 56, 24),
            (52, 20, 66, 42),
            (52, 42, 66, 66),
            (24, 58, 56, 70),
            (14, 42, 28, 66),
            (14, 20, 28, 42),
            (24, 36, 56, 48),
        ]
        observed = []
        for x1, y1, x2, y2 in boxes:
            region = resized[y1:y2, x1:x2]
            ratio = float(np.count_nonzero(region)) / float(region.size)
            observed.append(1 if ratio > 0.10 else 0)

        best_digit = None
        best_matches = -1
        for digit, segments in self.SEGMENTS.items():
            matches = sum(1 for a, b in zip(observed, segments) if a == b)
            if matches > best_matches:
                best_digit = digit
                best_matches = matches
        confidence = float(best_matches) / 7.0
        return best_digit, confidence

    def dark_digit_mask(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros(gray.shape, dtype=np.uint8)
        mask[(gray <= self.dark_threshold) & (hsv[:, :, 2] >= 10)] = 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), dtype=np.uint8))
        return mask

    def normalize_digit_mask(self, mask, size):
        _, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        coords = cv2.findNonZero(binary)
        if coords is None:
            return None
        x, y, w, h = cv2.boundingRect(coords)
        if w < 2 or h < 2:
            return None
        crop = binary[y : y + h, x : x + w]
        scale = min((size * 0.78) / float(w), (size * 0.78) / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        canvas = np.zeros((size, size), dtype=np.uint8)
        x0 = int(round((size - new_w) * 0.5))
        y0 = int(round((size - new_h) * 0.5))
        canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
        return canvas

    def iou(self, a, b):
        aa = a > 0
        bb = b > 0
        inter = float(np.count_nonzero(aa & bb))
        union = float(np.count_nonzero(aa | bb))
        if union <= 0.0:
            return 0.0
        return inter / union

    def find_contours(self, mask):
        result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(result) == 3:
            return result[1]
        return result[0]

    def nms(self, detections, max_iou):
        kept = []
        for det in sorted(detections, key=lambda item: item["confidence"], reverse=True):
            duplicate = False
            for prev in kept:
                if self.box_iou(det, prev) > max_iou:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(det)
        return sorted(kept, key=lambda item: (item["class_id"], -item["confidence"]))

    def box_iou(self, a, b):
        ix1 = max(a["xmin"], b["xmin"])
        iy1 = max(a["ymin"], b["ymin"])
        ix2 = min(a["xmax"], b["xmax"])
        iy2 = min(a["ymax"], b["ymax"])
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = float(iw * ih)
        area_a = float(max(0, a["xmax"] - a["xmin"]) * max(0, a["ymax"] - a["ymin"]))
        area_b = float(max(0, b["xmax"] - b["xmin"]) * max(0, b["ymax"] - b["ymin"]))
        union = area_a + area_b - inter
        if union <= 0.0:
            return 0.0
        return inter / union

    def update_stability(self, detections):
        self.frame_index += 1
        new_tracks = {}
        for det in detections:
            key = det["class_id"]
            prev = self.tracks.get(key)
            if prev is None:
                count = 1
            else:
                dx = det["center_x"] - prev["center_x"]
                dy = det["center_y"] - prev["center_y"]
                if math.sqrt(dx * dx + dy * dy) <= self.confirm_distance_px:
                    count = prev["count"] + 1
                else:
                    count = 1
            det["stable"] = count >= self.confirm_frames
            new_tracks[key] = {"center_x": det["center_x"], "center_y": det["center_y"], "count": count}
        self.tracks = new_tracks

    def add_world_center(self, det, shape):
        center_world = self.pixel_to_world(det["center_x"], det["center_y"], shape)
        if center_world is None:
            det["center_world_x"] = None
            det["center_world_y"] = None
            return
        det["center_world_x"], det["center_world_y"] = center_world

    def pixel_to_world(self, u, v, shape):
        height, width = shape[:2]
        x_min, x_max, y_min, y_max = self.view_bounds
        usable_w = float(width) - 2.0 * self.view_margin_px
        usable_h = float(height) - 2.0 * self.view_margin_px
        if usable_w <= 1.0 or usable_h <= 1.0:
            return None
        x = x_min + (float(u) - self.view_margin_px) * (x_max - x_min) / usable_w
        y = y_min + (float(height) - self.view_margin_px - float(v)) * (y_max - y_min) / usable_h
        return x, y

    def publish_results(self, header, detections):
        array_msg = Detection2DArray()
        array_msg.header = header
        for det in detections:
            msg = Detection2D()
            msg.class_id = int(det["class_id"])
            msg.class_name = str(det["class_name"])
            msg.confidence = float(det["confidence"])
            msg.xmin = int(det["xmin"])
            msg.ymin = int(det["ymin"])
            msg.xmax = int(det["xmax"])
            msg.ymax = int(det["ymax"])
            msg.center_x = int(det["center_x"])
            msg.center_y = int(det["center_y"])
            msg.yaw_deg = float(det.get("yaw_deg", 0.0))
            msg.stable = bool(det["stable"])
            array_msg.detections.append(msg)
        self.results_pub.publish(array_msg)

    def publish_stats(self, header, elapsed_ms, detections):
        msg = String()
        msg.data = json.dumps(
            {
                "backend": "traditional",
                "method": self.method,
                "stamp": header.stamp.to_sec() if header.stamp else 0.0,
                "frame_id": header.frame_id,
                "elapsed_ms": elapsed_ms,
                "detections": len(detections),
                "stable_detections": len([det for det in detections if det["stable"]]),
                "classes": [int(det["class_id"]) for det in detections],
                "colors": [str(det.get("color_name", "unknown")) for det in detections],
                "yaw_deg": [round(float(det.get("yaw_deg", 0.0)), 2) for det in detections],
            },
            sort_keys=True,
        )
        self.stats_pub.publish(msg)

    def publish_details(self, header, detections):
        items = []
        for det in detections:
            center_world = None
            if det.get("center_world_x") is not None and det.get("center_world_y") is not None:
                center_world = [
                    round(float(det["center_world_x"]), 4),
                    round(float(det["center_world_y"]), 4),
                ]
            items.append(
                {
                    "digit": int(det["class_id"]),
                    "confidence": round(float(det["confidence"]), 4),
                    "color": str(det.get("color_name", "unknown")),
                    "color_digit": int(det.get("color_digit", -1)),
                    "color_confidence": round(float(det.get("color_confidence", 0.0)), 4),
                    "color_corrected": bool(det.get("color_corrected", False)),
                    "center_px": [int(det["center_x"]), int(det["center_y"])],
                    "center_world_xy": center_world,
                    "yaw_deg": round(float(det.get("yaw_deg", 0.0)), 2),
                    "yaw_source": str(det.get("yaw_source", "unknown")),
                    "bbox": [
                        int(det["xmin"]),
                        int(det["ymin"]),
                        int(det["xmax"]),
                        int(det["ymax"]),
                    ],
                    "stable": bool(det["stable"]),
                }
            )
        msg = String()
        msg.data = json.dumps(
            {
                "stamp": header.stamp.to_sec() if header.stamp else 0.0,
                "frame_id": header.frame_id,
                "detections": items,
            },
            sort_keys=True,
        )
        self.details_pub.publish(msg)

    def draw_detections(self, image, detections):
        if self.use_detection_roi:
            height, width = image.shape[:2]
            x0 = int(round(max(0.0, min(1.0, self.roi_x_min_ratio)) * width))
            x1 = int(round(max(0.0, min(1.0, self.roi_x_max_ratio)) * width))
            y0 = int(round(max(0.0, min(1.0, self.roi_y_min_ratio)) * height))
            y1 = int(round(max(0.0, min(1.0, self.roi_y_max_ratio)) * height))
            cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (80, 180, 255), 2)
        for det in detections:
            color = (255, 160, 0) if det["stable"] else (255, 255, 0)
            cv2.rectangle(image, (det["xmin"], det["ymin"]), (det["xmax"], det["ymax"]), color, 2)
            cv2.circle(image, (det["center_x"], det["center_y"]), 4, color, -1)
            label = "%s:%s %s %.2f %s" % (
                self.method,
                det["class_name"],
                det.get("color_name", "unknown"),
                det["confidence"],
                "yaw %.0f %s" % (
                    float(det.get("yaw_deg", 0.0)),
                    "stable" if det["stable"] else "pending",
                ),
            )
            cv2.putText(
                image,
                label,
                (det["xmin"], max(16, det["ymin"] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        return image

    def show_image(self, image):
        if not self.show_window or self.window_failed:
            return
        try:
            if not self.window_ready:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.window_ready = True
            cv2.imshow(self.window_name, image)
            cv2.waitKey(1)
        except Exception as exc:
            self.window_failed = True
            rospy.logwarn("OpenCV window disabled: %s", exc)


def main():
    TraditionalDigitDetector()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
