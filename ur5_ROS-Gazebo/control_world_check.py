#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
控制端到端验证脚本：给定 world 坐标，让 EE 去那里，看实际偏差。

用法：
    rosrun ur5_notebook control_world_check.py _x:=0.601 _y:=0.165 _z:=0.30

流程：
    1. 接收 world 坐标 P_world = (x, y, z)
    2. 转 base_link 系：P_base = P_world - SPAWN_OFFSET = (x, y-0.7, z-0.2)
    3. MoveIt set_pose_target(P_base) + plan + execute
    4. 读吸盘真实位置（TF 树里 world == base_link 是 identity，
       所以 lookup world->vacuum_gripper 拿到的是 base 系数值，
       再加 SPAWN_OFFSET 还原成真 world）
    5. 也可选地从 /gazebo/link_states 拿 wrist_3_link 真值做交叉验证
"""

from __future__ import print_function

import math
import sys

import rospy
import moveit_commander
import tf
from copy import deepcopy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates


SPAWN_OFFSET_X = 0.0
SPAWN_OFFSET_Y = 0.7
SPAWN_OFFSET_Z = 0.2

# EE 朝下姿态（与 grasp_executor.py 同款）
DEFAULT_DOWN_ORIENTATION = (0.4811, 0.4994, -0.5121, 0.5069)


class ControlWorldCheck(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_world_check", anonymous=False)

        self.target_x = float(rospy.get_param("~x", 0.5))
        self.target_y = float(rospy.get_param("~y", 0.0))
        self.target_z = float(rospy.get_param("~z", 0.30))
        self.gripper_frame = rospy.get_param(
            "~gripper_frame", "vacuum_gripper"
        )
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.move_group_name = rospy.get_param("~move_group", "manipulator")
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.3))
        self.acceleration_scaling = float(
            rospy.get_param("~acceleration_scaling", 0.3)
        )

        self.link_states = None
        rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.link_states_cb, queue_size=1
        )
        self.tf_listener = tf.TransformListener()

        rospy.sleep(2.0)
        self.arm = moveit_commander.MoveGroupCommander(self.move_group_name)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.15)
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(5.0)
        self.ee_link = self.arm.get_end_effector_link()

        rospy.loginfo(
            "control_world_check: target world=(%.3f, %.3f, %.3f), ee_link=%s, "
            "gripper_frame(TF)=%s",
            self.target_x, self.target_y, self.target_z,
            self.ee_link, self.gripper_frame,
        )

    def link_states_cb(self, msg):
        self.link_states = msg

    def gripper_world_via_tf(self):
        """通过 TF 拿吸盘位置。
        TF 里 world->base_link 是 identity，所以 world->vacuum_gripper 实际就是
        base 系下的吸盘位置。加上 SPAWN_OFFSET 还原成物理 world。"""
        try:
            self.tf_listener.waitForTransform(
                self.world_frame, self.gripper_frame,
                rospy.Time(0), rospy.Duration(1.0),
            )
            (trans, _) = self.tf_listener.lookupTransform(
                self.world_frame, self.gripper_frame, rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("TF %s -> %s failed: %s",
                          self.world_frame, self.gripper_frame, exc)
            return None
        bx, by, bz = trans  # 这其实是 base 系坐标（伪 world）
        return (bx + SPAWN_OFFSET_X, by + SPAWN_OFFSET_Y, bz + SPAWN_OFFSET_Z)

    def wrist3_world_via_link_states(self):
        """从 link_states 拿 wrist_3_link 的真物理 world 位置（交叉验证）。"""
        if self.link_states is None:
            return None
        for name_candidate in ("robot::wrist_3_link", "wrist_3_link"):
            if name_candidate in self.link_states.name:
                idx = self.link_states.name.index(name_candidate)
                p = self.link_states.pose[idx].position
                return (p.x, p.y, p.z, name_candidate)
        return None

    def run(self):
        # 1. world -> base 转换
        bx = self.target_x - SPAWN_OFFSET_X
        by = self.target_y - SPAWN_OFFSET_Y
        bz = self.target_z - SPAWN_OFFSET_Z

        # 2. 构造 base 系 pose
        pose = Pose()
        pose.position.x = bx
        pose.position.y = by
        pose.position.z = bz
        pose.orientation.x = DEFAULT_DOWN_ORIENTATION[0]
        pose.orientation.y = DEFAULT_DOWN_ORIENTATION[1]
        pose.orientation.z = DEFAULT_DOWN_ORIENTATION[2]
        pose.orientation.w = DEFAULT_DOWN_ORIENTATION[3]

        rospy.loginfo(
            "Step 1/3: target_world=(%+.4f, %+.4f, %+.4f) -> "
            "target_base=(%+.4f, %+.4f, %+.4f)",
            self.target_x, self.target_y, self.target_z, bx, by, bz,
        )

        # 3. plan + execute
        rospy.loginfo("Step 2/3: planning + executing...")
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose, self.ee_link)
        plan = self.arm.plan()
        if isinstance(plan, tuple):
            ok = plan[0]
            traj = plan[1]
        else:
            traj = plan
            ok = bool(traj and traj.joint_trajectory.points)
        if not ok or not traj or not traj.joint_trajectory.points:
            rospy.logerr("planning FAILED")
            return

        success = self.arm.execute(traj, wait=True)
        self.arm.clear_pose_targets()
        if not success:
            rospy.logwarn("execute returned False, 但继续读实际位置")

        # 4. 等机械臂稳定再读位置
        rospy.loginfo("Step 3/3: settling 1.5s, reading actual positions...")
        rospy.sleep(1.5)

        gripper_world = self.gripper_world_via_tf()
        wrist3 = self.wrist3_world_via_link_states()

        print("\n" + "=" * 76)
        print("控制端到端验证结果（不经过任何视觉）")
        print("=" * 76)
        print("目标 world : (%+.4f, %+.4f, %+.4f)" %
              (self.target_x, self.target_y, self.target_z))

        if gripper_world is not None:
            ax, ay, az = gripper_world
            dx = ax - self.target_x
            dy = ay - self.target_y
            dz = az - self.target_z
            err_xy = math.hypot(dx, dy)
            print("吸盘 world : (%+.4f, %+.4f, %+.4f)  ← TF lookup + spawn_offset" %
                  (ax, ay, az))
            print("误差       : dx=%+.4f dy=%+.4f dz=%+.4f" % (dx, dy, dz))
            print("xy 平面误差: %.4f m  (%.1f mm)" % (err_xy, err_xy * 1000))
            print("z 高度误差 : %+.4f m  (%+.1f mm)" % (dz, dz * 1000))
        else:
            print("吸盘 world : (TF 不可用)")

        if wrist3 is not None:
            wx, wy, wz, wname = wrist3
            print("---")
            print("交叉验证 wrist_3_link world (来自 /gazebo/link_states '%s')"
                  % wname)
            print("           : (%+.4f, %+.4f, %+.4f)" % (wx, wy, wz))
            print("  (吸盘距 wrist_3 应约 5cm 沿 EE 轴向；用此判断 TF 数据是否合理)")
        print("=" * 76)
        print("解读：")
        print("  - 如果吸盘 xy 误差 < 5mm  → 控制闭环精准")
        print("  - 5~30mm → MoveIt + Gazebo PID 一般水平，足够吸盘 maxDistance(50mm) 工作")
        print("  - >50mm → 控制有问题（IK / TF / 偏移）")
        print("=" * 76)


def main():
    node = ControlWorldCheck()
    node.run()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
