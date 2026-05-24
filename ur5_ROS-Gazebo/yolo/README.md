# YOLOv8 数字物块检测

本目录保存 B 视觉检测模块使用的 YOLOv8 配置、数据列表、worker 脚本和权重占位文件。当前目标是识别右侧可抓取彩色数字物块 `0-9`，左侧白底目标格子不作为检测目标。

## 目录说明

- `data_yolov8.yaml`：Ultralytics YOLOv8 数据集配置，类别顺序固定为 `0-9`。
- `classes.names`：数字类别名称，供旧脚本或调试工具复用。
- `data/train_highres.txt`、`data/valid_highres.txt`：训练集和验证集图片列表。
- `data/images/`、`data/labels/`：Gazebo 生成的数据集图片和 YOLO 标签，默认不提交到 Git。
- `weights/best.pt`：训练完成后放置的 YOLOv8 推理权重，默认不提交到 Git。
- `yolov8_worker.py`：容器内 Python3 CPU 推理 worker，由 ROS Python2 wrapper 调用。

## 训练

训练在 Mac 宿主机执行，使用 Apple Silicon MPS 加速：

```bash
cd /Users/yingziyu009/Desktop/ur5_grasp_in_gazebo_v1
source .venv/bin/activate
python scripts/train_yolov8_digits.py \
  --data ur5_ROS-Gazebo/yolo/data_yolov8.yaml \
  --model yolov8n.pt \
  --epochs 120 \
  --imgsz 640 \
  --batch 8 \
  --device mps
```

脚本会检查 `torch.backends.mps` 是否可用，并加入适度亮度、饱和度、对比度、轻微模糊和小幅几何扰动。数字不能翻转，因此训练脚本关闭了水平和垂直翻转。

训练结束后，脚本会生成：

```bash
ur5_ROS-Gazebo/yolo/weights/best.pt
ur5_ROS-Gazebo/yolo/weights/best_training_full.pt
```

`best.pt` 用于 ROS 推理，`best_training_full.pt` 保留完整训练 checkpoint。两者都不提交到 Git，需要通过 Docker volume 或其他方式在本机和容器之间同步。

## 容器内推理

ROS Kinetic 使用 Python2 运行 `detection.py`，Ultralytics YOLOv8 在 `/opt/yolo_py` 的 Python3 环境里运行。重建镜像后检查依赖：

```bash
docker exec -it ur5_grasp_kinetic bash
/opt/yolo_py/bin/python -c "import torch, ultralytics, cv2; print(torch.__version__, ultralytics.__version__, cv2.__version__)"
```

检查 worker 能否加载模型：

```bash
/opt/yolo_py/bin/python \
  /root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/yolov8_worker.py \
  --self-test \
  --model /root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/weights/best.pt
```

启动检测：

```bash
source /root/arm_ws/devel/setup.bash
roslaunch ur5_notebook detection.launch
```

常用查看命令：

```bash
rostopic echo /detection/results
rostopic echo /detection/stats
export DISPLAY=:99
rqt_image_view /detection/annotated_image
```

`Detection2DArray` 中每个目标包含类别、置信度、bbox、中心点、`yaw_deg` 旋转角和多帧稳定标志。
