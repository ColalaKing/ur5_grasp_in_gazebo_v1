# Pose Estimation Module Placeholder

Owner: C module.

Input expected from basic function 2:

- `/detection/results` (`ur5_notebook/Detection2DArray`)
- `/ur5/wrist_camera/camera_info` (`sensor_msgs/CameraInfo`)
- TF frame `wrist_camera_optical_frame`

Expected output for later motion planning:

- Target grasp poses in the robot base frame.

This folder is intentionally left as a handoff point. It should not change the YOLO detector output contract.
