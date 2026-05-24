#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

DEFAULT_VIEW_POSE = [-1.57691, -1.71667, 1.79266, -1.67721, -1.57050, 0.0]


def parse_positions(value):
    if isinstance(value, list):
        positions = [float(item) for item in value]
    else:
        tokens = [item for item in re.split(r"[\s,\[\]]+", str(value)) if item]
        positions = [float(item) for item in tokens]
    if len(positions) != 6:
        raise ValueError("joint_positions must contain exactly 6 values")
    return positions


def main():
    rospy.init_node("wrist_camera_view_pose", anonymous=False)

    topic = rospy.get_param("~controller_topic", "/arm_controller/command")
    positions = parse_positions(rospy.get_param("~joint_positions", DEFAULT_VIEW_POSE))
    duration = float(rospy.get_param("~duration", 4.0))
    repeat = int(rospy.get_param("~repeat", 5))

    pub = rospy.Publisher(topic, JointTrajectory, queue_size=1)
    deadline = rospy.Time.now() + rospy.Duration(20.0)
    while pub.get_num_connections() == 0 and rospy.Time.now() < deadline and not rospy.is_shutdown():
        rospy.loginfo_throttle(2.0, "Waiting for arm controller topic %s" % topic)
        rospy.sleep(0.2)

    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * 6
    point.time_from_start = rospy.Duration(duration)
    traj.points.append(point)

    rate = rospy.Rate(4.0)
    for _ in range(max(1, repeat)):
        if rospy.is_shutdown():
            break
        traj.header.stamp = rospy.Time.now()
        pub.publish(traj)
        rate.sleep()

    rospy.loginfo("Commanded wrist camera view pose: %s", positions)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("wrist_camera_view_pose failed: %s", exc)
        raise
