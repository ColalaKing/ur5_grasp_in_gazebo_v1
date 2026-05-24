# YOLOv8 权重目录

训练完成后，把 ROS 推理用权重放在本目录：

```bash
best.pt
```

训练脚本默认会自动复制到：

```bash
ur5_ROS-Gazebo/yolo/weights/best.pt
```

权重文件属于二进制训练产物，默认不提交到 Git。容器运行时通过 Docker volume 读取宿主机仓库里的同一路径。
