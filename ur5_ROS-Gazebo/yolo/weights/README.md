# YOLOv8 权重目录

训练完成后，把 ROS 推理用权重放在本目录：

```bash
best.pt
```

训练脚本默认会自动复制到：

```bash
ur5_ROS-Gazebo/yolo/weights/best.pt
```

为方便组内后续抓取、坐标转换和集成测试，当前的 `best.pt` 已随功能分支提交到 GitHub。重新训练产生的其他 checkpoint，例如 `best_training_full.pt` 和中间权重，仍然不要提交。
