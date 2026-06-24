#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
2D->3D 映射 + 控制端到端 双重验收脚本（只读，不影响主流程）。

订阅：
  /localization/grasp_targets  (block_localization 的推理输出)
  /gazebo/model_states         (Gazebo 物理引擎给的真值)

两套指标：
  A. 像素 -> 3D 映射精度
     比较 block_localization 输出的 pose_world 与 model_states 真值
     目标：每块方块 xy 误差 < 1 mm
  B. 控制端到端精度（grasp_executor 跑过抓取后用）
     如果用户传入参数 ~check_block_id（如 ~check_block_id:=2），脚本会持续
     监测该方块 world 位置变化，看是否被搬到了 grid_X：
       - 抓取成功的判断：block 当前 world.xy 距 grid_X 的 world.xy < 5cm

用法：
    rosrun ur5_notebook localization_truth_diff.py
    rosrun ur5_notebook localization_truth_diff.py _check_block_id:=2
"""

from __future__ import print_function

import math
import rospy
from gazebo_msgs.msg import ModelStates
from ur5_notebook.msg import GraspTargetArray


# 目标格子在物理 world 系下的位置（与 grasp_executor.grid_pose_world 同公式）
GRID_ORIGIN_X = -0.25
GRID_ORIGIN_Y = -0.22
GRID_SPACING_X = 0.18
GRID_SPACING_Y = 0.18
GRID_COLS = 3


def grid_pose_world(grid_id):
    """grid_id 1..6 对应物理 world (x, y)"""
    idx = int(grid_id) - 1
    x = GRID_ORIGIN_X + (idx % GRID_COLS) * GRID_SPACING_X
    y = GRID_ORIGIN_Y + (idx // GRID_COLS) * GRID_SPACING_Y
    return x, y


class LocalizationTruthDiff(object):
    def __init__(self):
        rospy.init_node("localization_truth_diff", anonymous=True)
        self.print_period = float(rospy.get_param("~print_period", 2.0))
        self.spawn_offset_x = float(rospy.get_param("~spawn_offset_x", 0.0))
        self.spawn_offset_y = float(rospy.get_param("~spawn_offset_y", 0.7))
        self.spawn_offset_z = float(rospy.get_param("~spawn_offset_z", 0.2))
        self.check_block_id = int(rospy.get_param("~check_block_id", -1))
        # ~once:=true 时打印一次就退出，便于一次性截屏验收
        self.once_mode = str(rospy.get_param("~once", "false")).lower() in (
            "1", "true", "yes", "on"
        )
        self.printed_once = False

        self.last_truth = {}     # class_id -> (x_world, y_world, z_world)
        self.last_targets = {}   # class_id -> GraspTarget
        self.last_print_t = 0.0

        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.truth_cb, queue_size=1)
        rospy.Subscriber("/localization/grasp_targets", GraspTargetArray,
                         self.targets_cb, queue_size=1)
        rospy.loginfo("localization_truth_diff started, print every %.1fs, "
                      "check_block_id=%d", self.print_period, self.check_block_id)

    def truth_cb(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if not name.startswith("digit_block_"):
                continue
            try:
                cid = int(name.replace("digit_block_", ""))
            except ValueError:
                continue
            self.last_truth[cid] = (pose.position.x, pose.position.y,
                                    pose.position.z)
        self.maybe_print()

    def targets_cb(self, msg):
        for t in msg.targets:
            self.last_targets[t.class_id] = t
        self.maybe_print()

    def maybe_print(self):
        now = rospy.get_time()
        if now - self.last_print_t < self.print_period:
            return
        self.last_print_t = now

        if not self.last_truth or not self.last_targets:
            return

        self._print_mapping_table()
        if self.check_block_id >= 0:
            self._print_grasp_check()

        if self.once_mode and not self.printed_once:
            self.printed_once = True
            rospy.loginfo("once mode: print done, shutting down.")
            rospy.signal_shutdown("once mode complete")

    def _print_mapping_table(self):
        print("\n" + "=" * 100)
        print("[A] 像素 -> 3D 映射精度对比 (单位: m)")
        print("=" * 100)
        print("%-4s | %-26s | %-26s | %-26s | %-9s | %-12s"
              % ("cls", "Gazebo truth (world)",
                 "推理 pose_world",
                 "推理 pose_base (base_link)",
                 "err_world", "err_base_check"))
        print("-" * 100)

        max_err_world = 0.0
        max_err_base = 0.0
        n = 0
        for cid in sorted(self.last_targets.keys()):
            t = self.last_targets[cid]
            truth = self.last_truth.get(cid)
            if truth is None:
                continue
            tx, ty, tz = truth
            wx, wy = t.pose_world.position.x, t.pose_world.position.y
            bx, by = t.pose_base.position.x, t.pose_base.position.y

            # err_world：直接对比推理 world vs 真值
            err_world = math.hypot(wx - tx, wy - ty)

            # err_base_check：把 pose_base 加上 spawn 偏移还原成 world，再和真值比较
            # 如果 pose_base 是"真 base 系"，加偏移后应等于真 world，err_base_check 应≈0
            # 如果 pose_base 数值还是 world（旧版 bug），加偏移就多了一个 spawn，err≈0.7
            bx_w = bx + self.spawn_offset_x
            by_w = by + self.spawn_offset_y
            err_base = math.hypot(bx_w - tx, by_w - ty)

            max_err_world = max(max_err_world, err_world)
            max_err_base = max(max_err_base, err_base)
            n += 1

            print("%-4d | (%+.4f,%+.4f,%+.4f) | (%+.4f,%+.4f,%+.4f) | "
                  "(%+.4f,%+.4f,%+.4f) | %7.4f  | %7.4f"
                  % (cid, tx, ty, tz, wx, wy, t.pose_world.position.z,
                     bx, by, t.pose_base.position.z, err_world, err_base))

        print("-" * 100)
        print("汇总: 共 %d 个方块" % n)
        if n > 0:
            print("  - err_world  最大 = %.4f m (%.2f mm)  ← 像素->world 精度"
                  % (max_err_world, max_err_world * 1000))
            print("  - err_base_check 最大 = %.4f m (%.2f mm)"
                  % (max_err_base, max_err_base * 1000))
        verdict_a = "PASS ✅" if max_err_world < 0.005 else "FAIL ❌"
        verdict_b = ("PASS ✅"
                     if max_err_base < 0.005
                     else "FAIL ❌ (pose_base 没在 base 系下)")
        print("[A1] 像素->world 映射精度 (err_world < 5mm): %s" % verdict_a)
        print("[A2] pose_base 真在 base_link 系 (err_base_check < 5mm): %s"
              % verdict_b)
        print("=" * 100)

    def _print_grasp_check(self):
        cid = self.check_block_id
        truth = self.last_truth.get(cid)
        if truth is None:
            return
        if not (1 <= cid <= 6):
            return
        gx, gy = grid_pose_world(cid)
        tx, ty, _ = truth
        dist = math.hypot(tx - gx, ty - gy)
        print("\n" + "=" * 100)
        print("[B] 控制端到端：block_%d 是否被抓到 grid_%d？" % (cid, cid))
        print("=" * 100)
        print("  block_%d 当前 world: (%+.4f, %+.4f)" % (cid, tx, ty))
        print("  grid_%d  目标 world: (%+.4f, %+.4f)" % (cid, gx, gy))
        print("  距离 grid 中心     : %.4f m (%.1f mm)" % (dist, dist * 1000))
        verdict = "PASS ✅ (已放到位)" if dist < 0.05 else "PENDING (未放到位)"
        print("[B] 抓取放置精度 (dist < 5cm): %s" % verdict)
        print("=" * 100)


def main():
    LocalizationTruthDiff()
    rospy.spin()


if __name__ == "__main__":
    main()
