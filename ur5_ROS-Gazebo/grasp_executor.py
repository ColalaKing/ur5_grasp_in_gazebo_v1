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
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from std_srvs.srv import EmptyResponse

from ur5_notebook.msg import GraspTarget
from ur5_notebook.msg import GraspTargetArray


DEFAULT_DOWN_ORIENTATION = (0.4811, 0.4994, -0.5121, 0.5069)

# block_localization 现在已经把 pose_base 真正转换到 base_link 系，
# 所以 grasp_executor 不再需要对它手动减 spawn 偏移。
# 但仍保留 GAZEBO_SPAWN_* 给 transform_world_pose_to_base 使用——
# 那个函数的输入是 grasp_executor 自己手算的 grid world 坐标（真物理 world），
# 转到 base 系仍需减 spawn 偏移。
SPAWN_OFFSET_X = 0.0  # 视觉发出来的 pose_base 不需要再修正
SPAWN_OFFSET_Y = 0.0
SPAWN_OFFSET_Z = 0.0

GAZEBO_SPAWN_X = 0.0
GAZEBO_SPAWN_Y = 0.7
GAZEBO_SPAWN_Z = 0.2

# UR5 base_link 系下水平最大可达半径（保守值，留余量）
UR5_REACH_XY = 0.80
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
        # gazebo_ros_vacuum_gripper plugin 按 link 中心距离判定吸附：
        # minDistance=0.01, maxDistance=0.05；让 EE 在方块中心上方 30mm（中位数），
        # vacuum dist=30mm 落在工作区间内，且不会刚体穿透方块导致 controller 卡死。
        self.grasp_z_offset = float(rospy.get_param("~grasp_z_offset", 0.050))
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
            rospy.get_param("~max_velocity_scaling", 0.2)
        )
        self.max_acceleration_scaling = float(
            rospy.get_param("~max_acceleration_scaling", 0.2)
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
        self.latest_reachable = {}
        self.busy = False
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()

        # Direct attach: when vacuum is on, we manually set the block model state
        # to follow the vacuum_gripper position via /gazebo/set_model_state.
        # vacuum plugin in this env does not detect contact reliably, so we drive it.
        self.attach_active = False
        self.attach_block_id = -1
        self.attach_block_quat = (0.0, 0.0, 0.0, 1.0)
        # EE→Block 相对旋转：吸住瞬间记录，之后每帧用当前 EE quat × rel 还原方块姿态。
        # 这样 EE 在抬升/横移时旋转 wrist_3，方块会跟着一起转，视觉上等同物理 attach。
        self.attach_relative_quat = (0.0, 0.0, 0.0, 1.0)
        self.attach_thread = None
        self.set_model_state_proxy = None

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
        reachable = {}
        for target in msg.targets:
            if self.require_stable and not target.stable:
                continue
            cid = int(target.class_id)
            targets[cid] = target
            # target.pose_base 已经是真 base_link 系坐标，直接判工作半径。
            bx = target.pose_base.position.x
            by = target.pose_base.position.y
            if math.hypot(bx, by) <= UR5_REACH_XY:
                reachable[cid] = target
        with self.lock:
            self.latest_targets = targets
            self.latest_reachable = reachable

    def command_callback(self, msg):
        class_id = int(msg.data)
        if self.busy:
            self.publish_status("busy", "grasp already in progress")
            return
        # class_id == -1：自动挑一个可达且可放置（1~6）的目标
        if class_id == -1:
            with self.lock:
                placeable = sorted(
                    [c for c in self.latest_reachable.keys() if 1 <= c <= 6]
                )
            if not placeable:
                self.publish_status(
                    "rejected",
                    "no reachable target in [1..6] (only 1~6 have grid slots)",
                )
                return
            class_id = placeable[0]
            self.publish_status(
                "info", "auto-selected reachable class %d" % class_id
            )
        # class 0/7/8/9 没有 grid 槽位，抓起来没地方放，会掉到桌外，直接拒绝
        if class_id not in range(1, 7):
            self.publish_status(
                "rejected",
                "class %d has no grid slot (only 1~6 are placeable)" % class_id,
            )
            return
        # 该 class 不在视野内可达半径，拒绝并报告可达列表
        with self.lock:
            unreachable = (
                class_id in self.latest_targets
                and class_id not in self.latest_reachable
            )
            reachable_list = sorted(self.latest_reachable.keys())
        if unreachable:
            self.publish_status(
                "rejected",
                "class %d is out of UR5 reach. reachable classes: %s"
                % (class_id, reachable_list),
            )
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
        self.detach_block()
        self.set_gripper(False)
        self.busy = False
        self.publish_status("stopped", "motion stopped and gripper released")
        return EmptyResponse()

    def get_target(self, class_id):
        with self.lock:
            # 优先用可达表，命令分发时已经过滤过；这里再保险一层
            return self.latest_reachable.get(class_id) or self.latest_targets.get(class_id)

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
        # target.pose_base 现在是真正 base_link 系的坐标（block_localization 已显式
        # 减过 spawn 偏移），这里 SPAWN_OFFSET_* 都是 0，等于 noop，保留以防回归。
        pose.position.x -= SPAWN_OFFSET_X
        pose.position.y -= SPAWN_OFFSET_Y
        pose.position.z -= SPAWN_OFFSET_Z
        pose.position.z += z_offset
        qx, qy, qz, qw = self.build_tool_orientation(target.yaw_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def transform_world_pose_to_base(self, world_pose):
        # 输入：grid 等手算的物理 world 坐标。
        # 输出：base_link 系坐标（手动减 spawn 偏移，因为 URDF world->base_link 是
        #       identity，tf.transformPose 不会真减偏移）。
        pose = deepcopy(world_pose)
        pose.position.x -= GAZEBO_SPAWN_X
        pose.position.y -= GAZEBO_SPAWN_Y
        pose.position.z -= GAZEBO_SPAWN_Z
        return pose

    def grid_pose_world(self, grid_id):
        index = int(grid_id) - 1
        pose = Pose()
        pose.position.x = self.grid_origin_x + (index % self.grid_cols) * self.grid_spacing_x
        pose.position.y = self.grid_origin_y + (index // self.grid_cols) * self.grid_spacing_y
        pose.position.z = (
            self.table_z + self.table_size_z * 0.5 + self.block_size_z * 0.5 + 0.045
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

    def _ensure_set_model_state_proxy(self):
        if self.set_model_state_proxy is not None:
            return True
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            self.set_model_state_proxy = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )
            return True
        except rospy.ROSException as exc:
            rospy.logwarn("/gazebo/set_model_state unavailable: %s", exc)
            return False

    def _attach_loop(self):
        """Continuously snap digit_block_<id> world pose to the EE position.
        Runs while self.attach_active is True.

        We use MoveIt get_current_pose(ee_link) instead of TF lookup, because
        TF cache may serve stale transforms in this environment."""
        rate = rospy.Rate(50)
        block_name = "digit_block_%d" % self.attach_block_id
        # Block is locked 25mm below EE (just under the suction cup).
        block_z_below_ee = 0.025
        while self.attach_active and not rospy.is_shutdown():
            try:
                ee_ps = self.arm.get_current_pose(self.end_effector_link)
                ex = ee_ps.pose.position.x
                ey = ee_ps.pose.position.y
                ez = ee_ps.pose.position.z
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0, "get_current_pose failed: %s" % (exc,)
                )
                rate.sleep()
                continue

            # ee_ps.frame_id is base_link; convert to physical world by adding
            # the Gazebo spawn offset (URDF world->base_link is identity).
            ms = ModelState()
            ms.model_name = block_name
            ms.pose.position.x = ex + 0.0
            ms.pose.position.y = ey + 0.7
            ms.pose.position.z = ez + 0.2 - block_z_below_ee
            # 用当前 EE quat × 初始相对 quat 还原方块姿态，让方块跟随 EE 旋转。
            ee_q_world = (
                ee_ps.pose.orientation.x,
                ee_ps.pose.orientation.y,
                ee_ps.pose.orientation.z,
                ee_ps.pose.orientation.w,
            )
            block_q = tft.quaternion_multiply(
                ee_q_world, self.attach_relative_quat
            )
            ms.pose.orientation.x = block_q[0]
            ms.pose.orientation.y = block_q[1]
            ms.pose.orientation.z = block_q[2]
            ms.pose.orientation.w = block_q[3]
            # ★ 必须显式清零 twist；ModelState 默认会把物理引擎累积的
            #   linear / angular velocity 保留，导致即使 pose 被锁定，
            #   方块仍会以累积的角速度继续旋转（视觉上：原地自旋）。
            ms.twist.linear.x = 0.0
            ms.twist.linear.y = 0.0
            ms.twist.linear.z = 0.0
            ms.twist.angular.x = 0.0
            ms.twist.angular.y = 0.0
            ms.twist.angular.z = 0.0
            ms.reference_frame = "world"
            try:
                self.set_model_state_proxy(ms)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(
                    2.0, "set_model_state failed: %s" % (exc,)
                )
            rate.sleep()

    def attach_block(self, class_id):
        """Use gazebo_ros_link_attacher to create a real fixed joint between
        the vacuum_gripper link and the digit_block. Physics-level rigid
        attachment, no visual sync issues."""
        try:
            rospy.wait_for_service("/link_attacher_node/attach", timeout=3.0)
            attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
            req = AttachRequest()
            req.model_name_1 = "robot"
            req.link_name_1 = "wrist_3_link"
            req.model_name_2 = "digit_block_%d" % int(class_id)
            req.link_name_2 = "base_link"
            resp = attach_srv(req)
            if resp.ok:
                self.attach_block_id = int(class_id)
                self.attach_active = True
                rospy.loginfo(
                    "[link_attacher] attached digit_block_%d to vacuum_gripper",
                    int(class_id),
                )
                return True
            else:
                rospy.logerr(
                    "[link_attacher] attach call returned ok=False for block_%d",
                    int(class_id),
                )
                return False
        except Exception as exc:
            rospy.logerr("[link_attacher] attach failed: %s", exc)
            return False
        self.attach_thread = threading.Thread(target=self._attach_loop)
        self.attach_thread.daemon = True
        self.attach_thread.start()
        rospy.loginfo("[attach] block_%d now follows vacuum_gripper", class_id)
        return True

    def detach_block(self):
        """Detach the fixed joint via gazebo_ros_link_attacher."""
        if not self.attach_active:
            return
        try:
            rospy.wait_for_service("/link_attacher_node/detach", timeout=2.0)
            detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
            req = AttachRequest()
            req.model_name_1 = "robot"
            req.link_name_1 = "wrist_3_link"
            req.model_name_2 = "digit_block_%d" % self.attach_block_id
            req.link_name_2 = "base_link"
            resp = detach_srv(req)
            rospy.loginfo(
                "[link_attacher] detached digit_block_%d (ok=%s)",
                self.attach_block_id, resp.ok,
            )
        except Exception as exc:
            rospy.logwarn("[link_attacher] detach failed: %s", exc)
        self.attach_active = False
        self.attach_block_id = -1

    def can_reach_pose(self, pose):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        self.arm.clear_pose_targets()
        if isinstance(plan, tuple):
            # 兼容不同 moveit_commander 返回类型
            plan = plan[1]
        return bool(plan and getattr(plan, "joint_trajectory", None) and plan.joint_trajectory.points)

    def move_to_pose(self, pose, label):
        # 等机器人停稳：上一段 execute(wait=True) 返回时 PID 还有残余速度，
        # 立刻读 current_state 会和 controller 的 stop_trajectory_duration 撞车，
        # 导致下一段 controller failed during execution。
        rospy.sleep(1.0)
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        if not plan:
            self.publish_status("failed", "%s planning failed" % label)
            return False
        success = self.arm.execute(plan, wait=True)
        self.arm.clear_pose_targets()
        if not success:
            self.publish_status("failed", "%s execution failed" % label)
            return False
        return True

    def move_cartesian(self, waypoints, label, slow=False):
        """slow 参数保留但已无效：Gazebo PID 在低速下反而更容易触发 path_tolerance，
        统一用正常速度，下降段距离短不会撞飞方块."""
        # cartesian 规划用 set_start_state_to_current_state 显式钉住起点。
        # 必须等上一段 execute 完全停稳（controller 的 stop_trajectory_duration=0.5
        # 加 PID 收敛尾巴），否则 plan 出来的起点和真正下发时的 current 错位，
        # 触发 "Invalid Trajectory: start point deviates" 直接 ABORT。
        # 0.6s 在某些种子下不够，1.5s 给足余量；多花的时间无害。
        rospy.sleep(1.5)
        self.arm.set_start_state_to_current_state()
        self.arm.set_max_velocity_scaling_factor(self.max_velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.max_acceleration_scaling)
        plan, fraction = self.arm.compute_cartesian_path(
            waypoints, 0.01, 0.0, True
        )
        if fraction < self.cartesian_min_fraction:
            self.publish_status(
                "failed",
                "%s cartesian path incomplete (fraction=%.2f)" % (label, fraction),
            )
            return False
        # 诊断：打印 plan 里第一个点和最后一个点的 joint 值，方便对比 actual_state
        try:
            pts = plan.joint_trajectory.points
            if pts:
                rospy.loginfo(
                    "[diag] %s plan: %d pts, dur=%.2fs, start=%s, end=%s",
                    label, len(pts),
                    pts[-1].time_from_start.to_sec(),
                    ["%.3f" % v for v in pts[0].positions],
                    ["%.3f" % v for v in pts[-1].positions],
                )
                cur = self.arm.get_current_joint_values()
                rospy.loginfo(
                    "[diag] %s current_joint=%s",
                    label, ["%.3f" % v for v in cur],
                )
        except Exception as exc:
            rospy.logwarn("[diag] plan dump failed: %s", exc)
        success = self.arm.execute(plan, wait=True)
        if not success:
            # execute 失败大多是机械臂真没到位（Gazebo abort），
            # 继续后面流程会导致空抓。直接当失败处理。
            self.publish_status(
                "failed",
                "%s cartesian execute aborted (controller failed)" % label,
            )
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
            # vacuum plugin 工作距离 [10, 50]mm，所以两个下降目标都在这个区间：
            #   pre_grasp = +0.045（上限附近，先到达 vacuum 触发距离边缘）
            #   grasp     = +0.030（中位数，稳定吸附）
            # 这样 EE 不会撞进方块 → controller 不会 30s 卡死。
            pre_grasp_pose = self.make_pose(target, 0.045)
            grasp_pose = self.make_pose(target, self.grasp_z_offset)
            retreat_pose = self.make_pose(target, self.retreat_z_offset)

            # 先回到 named "home" / "up" 命名姿态，保证每次抓取从已知良好状态出发。
            # 不强制成功（如果 SRDF 没定义 named target，会在 except 里跳过）。
            try:
                self.arm.set_start_state_to_current_state()
                self.arm.set_named_target("up")
                self.arm.go(wait=True)
                self.arm.stop()
                rospy.sleep(0.3)
            except Exception as exc:
                rospy.logwarn("[home] cannot return to up named pose: %s", exc)

            if not self.can_reach_pose(approach_pose):
                self.publish_status(
                    "rejected",
                    "class %d unreachable (no valid approach plan)" % class_id,
                    {"class_id": class_id},
                )
                return

            if not self.move_to_pose(approach_pose, "approach"):
                return

            # 第一段：approach -> pre_grasp（正常速度）
            self.publish_status("descending", "pre-descend to class %d top" % class_id)
            if not self.move_cartesian(
                [deepcopy(pre_grasp_pose)], "pre_descend"
            ):
                return

            # 第二段：pre_grasp -> grasp（慢速，避免把方块撞飞）
            self.publish_status("contacting", "slow contact onto class %d" % class_id)
            if not self.move_cartesian(
                [deepcopy(grasp_pose)], "descend", slow=True
            ):
                return

            self.publish_status("vacuum_on", "vacuum ON for class %d" % class_id)
            # 诊断打印：vacuum_gripper link 当前世界位置 vs 方块世界位置
            # 让我们直接看到吸盘是否真的贴在方块表面
            try:
                self.tf_listener.waitForTransform(
                    self.world_frame, "vacuum_gripper", rospy.Time(0), rospy.Duration(0.5)
                )
                (vx, vy, vz), _ = self.tf_listener.lookupTransform(
                    self.world_frame, "vacuum_gripper", rospy.Time(0)
                )
                # 注意：URDF 里 world->base_link 是 identity，所以这里 TF 拿到的
                # "world" 数值实际上是 base_link 系。target.pose_base 也是 base 系，
                # 两边同系相减，得到吸盘和方块在 base 系下的距离。
                tx = target.pose_base.position.x
                ty = target.pose_base.position.y
                tz = target.pose_base.position.z
                dx, dy, dz = vx - tx, vy - ty, vz - tz
                rospy.loginfo(
                    "[diag] vacuum_gripper base=(%+.3f, %+.3f, %+.3f), "
                    "block_%d base=(%+.3f, %+.3f, %+.3f), "
                    "delta=(%+.3f, %+.3f, %+.3f), dist=%.3f",
                    vx, vy, vz, class_id, tx, ty, tz,
                    dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz),
                )
            except Exception as exc:
                rospy.logwarn("[diag] cannot lookup vacuum_gripper TF: %s", exc)
            self.set_gripper(True)
            # Force-attach the block to the gripper via /gazebo/set_model_state.
            # vacuum plugin alone does not reliably grasp in this env.
            self.attach_block(class_id)
            # 多等一会儿让真空吸力建立稳固
            rospy.sleep(max(self.gripper_wait, 1.5))

            # 抬升也用慢速，避免吸盘瞬间断开
            self.publish_status("retreating", "lifting class %d" % class_id)
            if not self.move_cartesian(
                [deepcopy(retreat_pose)], "retreat", slow=True
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
                            self.detach_block()
                            self.set_gripper(False)
                            rospy.sleep(1.5)
                            self.move_to_pose(place_retreat, "place retreat")
                            placed = True

            if not placed:
                self.detach_block()
                self.set_gripper(False)

            # 任务结束后回到 named "up" 姿态，方便下次抓取 / 多次连续操作。
            try:
                self.publish_status(
                    "returning_home",
                    "returning to home (up) pose",
                    {"class_id": class_id},
                )
                self.arm.set_start_state_to_current_state()
                self.arm.set_named_target("up")
                self.arm.go(wait=True)
                self.arm.stop()
                rospy.sleep(0.3)
            except Exception as exc:
                rospy.logwarn("[home] cannot return to up named pose: %s", exc)

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
