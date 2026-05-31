# UR5 大作业 Gazebo 仿真环境

本项目提供 UR5 机械臂在 ROS Kinetic + Gazebo + MoveIt 中的仿真环境，当前重点是大作业的环境搭建部分：机械臂、真空吸盘末端、腕部相机、桌面、0-9 数字方块、1-6 目标格子、随机方块初始位置和可选障碍物。

项目推荐使用 Docker 运行。宿主机不需要单独安装 ROS Kinetic、Gazebo 或 MoveIt。

容器内工作区路径：

```bash
/root/arm_ws
```

源码在容器内挂载到：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo
```

## 准备环境

### 1. 安装基础工具

在宿主机执行：

```bash
sudo apt update
sudo apt install -y git ca-certificates curl gnupg
```

### 2. 安装 Docker 和 Docker Compose

如果已经安装 Docker，可以先检查：

```bash
docker --version
docker compose version
docker ps
```

如果没有 Docker，可按 Docker 官方源安装：

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

把当前用户加入 Docker 用户组，避免每次输入 `sudo docker`：

```bash
sudo usermod -aG docker $USER
newgrp docker
```

### 3. 进入项目目录

```bash
cd ~/isaacsim/ur5_grasp_in_gazebo
```

如果你的仓库在其他路径，进入自己的仓库根目录即可。

## 启动并进入容器

CPU 和 GPU 只需要选择一种路线。第一次启动会构建镜像，时间会稍长。

### CPU 路线

适合没有 NVIDIA GPU，或者 GPU Docker/OpenGL 不稳定的电脑。

```bash
./scripts/start_gazebo_docker.sh
```

CPU 路线如果直接用 `gui:=true` 打不开 Gazebo 图形界面，建议使用下面的 CPU + VNC 方式。

### CPU + VNC 图形界面

CPU + VNC 会在容器内启动一个虚拟桌面，通过软件渲染运行 Gazebo，适合没有 NVIDIA GPU、远程桌面 OpenGL 不稳定、或者 Docker X11 直连失败的情况。Mac Docker/Rosetta 下 Gazebo7 的 `gzclient` 容易崩溃，所以脚本默认不打开 Gazebo GUI，而是启用稳定的 `/ur5/wrist_camera/image_raw` 备用腕部相机发布器。

宿主机先安装 VNC 查看器：

```bash
sudo apt update
sudo apt install -y tigervnc-viewer
```

第一个终端进入 CPU 容器：

```bash
./scripts/start_gazebo_docker.sh
```

进入容器后启动 VNC 版 Gazebo GUI：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
/root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
```

宿主机另开一个终端连接 VNC：

```bash
vncviewer localhost:5901
```

VNC 默认启动当前大作业环境，等价于 `assignment_env.launch gui:=false randomize:=true virtual_wrist_camera:=true`。如果要修改启动参数，可以在容器内用环境变量控制：

```bash
GAZEBO_RANDOMIZE=false /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_SEED=42 /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_SPAWN_OBSTACLES=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_PAUSED=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_GUI=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_VIRTUAL_WRIST_CAMERA=false /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
```

默认 Docker Compose 把容器内 `5900` 映射到宿主机 `5901`。如果要换端口，需要同步修改 `docker-compose.yml` 的端口映射和脚本的 `GAZEBO_VNC_PORT`。

```bash
GAZEBO_VNC_PORT=5900 /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
vncviewer localhost:5901
```

### GPU 路线

先确认宿主机 NVIDIA 驱动正常：

```bash
nvidia-smi
```

首次需安装并启用 NVIDIA Container Toolkit：

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

然后启动 GPU 容器：

```bash
./scripts/start_gazebo_docker_gpu.sh
```

进入容器后可检查 GPU/OpenGL：

```bash
nvidia-smi
glxinfo -B
```

`glxinfo -B` 中应看到 NVIDIA renderer。如果显示 `swrast`，说明 GPU OpenGL 没接好，可以先改用 CPU 路线。


## 编译工作区

进入容器后执行：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
rospack profile
rospack find ur5_notebook
```

如果配置正确，最后一条命令应输出：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo
```

以后重新进入容器终端后，通常只需要重新加载环境：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
```

## 功能命令

### 1. 随机生成方块初始位置

```bash
roslaunch ur5_notebook assignment_env.launch gui:=true randomize:=true
```

`seed:=0` ，表示每次启动使用当前时间作为随机种子，所以每次方块位置不同。
`seed:=42` ，设置随机种子，复现同一组随机方块位置。


### 2. 采用配置文件中的方块初始位置

```bash
roslaunch ur5_notebook assignment_env.launch gui:=true randomize:=false
```

固定布局适合视觉、坐标转换和演示录屏前调试。
建议只手动拖动方块。格子、桌面和机械臂是定位与控制基准，拖动它们会导致目标格子坐标、抓取高度或 MoveIt 当前状态不一致。

### 3. 开启可选障碍物

```bash
roslaunch ur5_notebook assignment_env.launch gui:=true randomize:=true spawn_obstacles:=true
```

障碍物只作为选做环境元素。若控制端要做避障，还需要把障碍物同步加入 MoveIt Planning Scene。

### 4. 查看腕部相机话题

仿真启动后，另开一个容器终端：

```bash
docker exec -it ur5_grasp_kinetic bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
rostopic list | grep -E "wrist_camera|camera"
```

正常会看到：

```bash
/ur5/wrist_camera/camera_info
/ur5/wrist_camera/image_raw
```

查看图像频率：

```bash
rostopic hz /ur5/wrist_camera/image_raw
```

查看相机参数：

```bash
rostopic echo -n 1 /ur5/wrist_camera/camera_info
```

如需打开图像窗口，先保证这个终端也使用 VNC 的显示号：

```bash
export DISPLAY=:99
rqt_image_view /ur5/wrist_camera/image_raw
```

当前主接口仍是腕部相机话题。`assignment_env.launch` 默认启用 `virtual_wrist_camera:=true`，用于绕过 Mac Docker/Rosetta 下 Gazebo7 相机传感器注册但不吐帧的问题；换到 Linux 原生或 GPU 环境后，如果真实 Gazebo 相机可以稳定发布帧，可用 `virtual_wrist_camera:=false` 关闭备用发布器。`assignment_env.launch` 也默认会发送一组观察姿态，让相机尽量正对桌面方块；如果后续运动规划同学需要完全手动控制机械臂，可以关闭：

```bash
roslaunch ur5_notebook assignment_env.launch gui:=false move_to_view_pose:=false
```

### 5. B 视觉检测功能概览

B 视觉检测部分负责识别右侧彩色数字物块，不把左侧 1-6 目标格子当成待抓取物块。当前实现包含两条检测路线：

- YOLOv8 主方案：识别 0-9 共 10 类数字方块，输出类别、置信度、bbox、中心点像素坐标和 `yaw_deg` 旋转角。
- 传统视觉对照方案：提供颜色、轮廓、模板和混合方法，方便与 YOLO 结果做准确率和耗时对比。

左侧目标格子只作为放置目标和背景参考，不在 YOLO 标签中标注。`detection.py` 还会做颜色饱和度过滤和检测区域约束，降低白底目标格子被误检为数字物块的概率。6 和 9 形状接近时，传统方法会结合物块颜色辅助区分，YOLO 训练数据也保留颜色差异作为有效特征。

### 6. 采集 YOLOv8 训练数据

先启动仿真环境：

```bash
roslaunch ur5_notebook assignment_env.launch gui:=false randomize:=true virtual_wrist_camera:=true
```

另开终端采集训练数据：

```bash
roslaunch ur5_notebook collect_yolo_dataset.launch sample_count:=300
```

想多采几组随机场景时，重启仿真并换一个 `seed`，后续采集用 `reset_train_list:=false` 追加到同一个训练列表：

```bash
roslaunch ur5_notebook assignment_env.launch gui:=false randomize:=true seed:=2 virtual_wrist_camera:=true
roslaunch ur5_notebook collect_yolo_dataset.launch sample_count:=300 reset_train_list:=false
```

生成的数据位于：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/data
```

Ultralytics 配置文件位于：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/data_yolov8.yaml
```

类别顺序固定为 0-9：

```bash
0 1 2 3 4 5 6 7 8 9
```

训练图片可以包含左侧目标格子，但标签只标右侧可抓取彩色数字物块。这样 YOLO 会把左侧白底数字格子学习成背景，避免后续抓取时把目标位置误认为物块。

### 7. 在 Mac 本机训练 YOLOv8

训练在 Mac 宿主机执行，使用 Ultralytics YOLOv8 和 Apple Silicon MPS 加速。建议先建一个 Python3 虚拟环境：

```bash
cd /Users/yingziyu009/Desktop/ur5_grasp_in_gazebo_v1
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install ultralytics torch torchvision opencv-python-headless albumentations matplotlib
```

开始训练：

```bash
python scripts/train_yolov8_digits.py \
  --data ur5_ROS-Gazebo/yolo/data_yolov8.yaml \
  --model yolov8n.pt \
  --epochs 120 \
  --imgsz 640 \
  --batch 8 \
  --device mps
```

训练脚本会先检查 MPS 是否可用，如果不可用会直接退出，避免误用 CPU 慢跑。脚本内置了适度数据增强：HSV 亮度/饱和度变化、轻微平移缩放透视、少量 mosaic，并在安装 `albumentations` 时额外加入轻微模糊和亮度/对比度变化。数字识别不能水平或垂直翻转，所以脚本显式关闭 `fliplr` 和 `flipud`。

训练完成后，脚本会把用于 ROS 推理的权重复制到：

```bash
/Users/yingziyu009/Desktop/ur5_grasp_in_gazebo_v1/ur5_ROS-Gazebo/yolo/weights/best.pt
```

完整训练 checkpoint 会另存为：

```bash
/Users/yingziyu009/Desktop/ur5_grasp_in_gazebo_v1/ur5_ROS-Gazebo/yolo/weights/best_training_full.pt
```

注意：`*.pt`、训练数据图片、标签和 `runs/` 目录不提交到 Git。Docker volumes 会把本机仓库同步到容器，所以容器内推理权重路径是：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/weights/best.pt
```

### 8. 构建容器内 YOLOv8 推理环境

ROS Kinetic 默认使用 Python2，Ultralytics 需要 Python3。为避免 `rospy/cv_bridge` 与 Python3 冲突，Dockerfile 中新增了独立 Python3 环境 `/opt/yolo_py`：ROS 节点仍用 Python2 订阅/发布话题，YOLOv8 推理由 Python3 worker 子进程完成。

修改 Dockerfile 后需要重建镜像：

```bash
docker compose build ur5_grasp
docker compose up -d
```

进入容器检查环境：

```bash
docker exec -it ur5_grasp_kinetic bash
/opt/yolo_py/bin/python -c "import torch, ultralytics, cv2; print(torch.__version__, ultralytics.__version__, cv2.__version__)"
```

检查 `best.pt` 是否已经通过 volume 出现在容器内：

```bash
ls -lh /root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/weights/best.pt
```

可选：直接测试 Python3 worker 是否能加载模型：

```bash
/opt/yolo_py/bin/python \
  /root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/yolov8_worker.py \
  --self-test \
  --model /root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/yolo/weights/best.pt
```

### 9. 启动 YOLOv8 目标检测节点

新增消息或脚本后需要先重新编译工作区：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
```

启动检测节点：

```bash
roslaunch ur5_notebook detection.launch
```

常用参数：

```bash
roslaunch ur5_notebook detection.launch \
  classes:=0,1,2,3,4,5,6,7,8,9 \
  input_size:=512 \
  confidence_threshold:=0.50 \
  iou_threshold:=0.45 \
  confirm_frames:=3 \
  worker_timeout:=2.0
```

`detection.py` 会订阅：

```bash
/ur5/wrist_camera/image_raw
```

并发布检测数组：

```bash
/detection/results
```

查看检测数组：

```bash
rostopic echo /detection/results
```

查看每帧耗时和类别统计：

```bash
rostopic echo /detection/stats
```

在 VNC 里查看标注后的图像：

```bash
export DISPLAY=:99
rqt_image_view /detection/annotated_image
```

检测输出字段包括 `class_id`、`class_name`、`confidence`、`xmin/ymin/xmax/ymax`、`center_x/center_y`、`yaw_deg` 和 `stable`。`stable=true` 表示同一类别目标已经通过多帧确认。`yaw_deg` 是根据数字物块彩色方块外轮廓估计的平面旋转角，后续坐标转换和抓取姿态可以直接使用这个字段。

### 10. 像素坐标转换为抓取坐标

启动检测节点后，另开一个容器终端启动几何转换节点：

```bash
roslaunch ur5_notebook grasp_pose_converter.launch
```

该节点订阅：

```bash
/detection/results
/ur5/wrist_camera/camera_info
```

并发布：

```bash
/grasp_targets/poses
/grasp_targets/selected
/grasp_targets/approach
/grasp_targets/details
```

其中 `/grasp_targets/selected` 是当前置信度最高的抓取点，默认发布在 `base_link` 坐标系下；`/grasp_targets/approach` 是同一目标上方 10cm 的预抓取点。查看结果：

```bash
rostopic echo /grasp_targets/selected
rostopic echo /grasp_targets/approach
rostopic echo /grasp_targets/details
```

`/grasp_targets/details` 会同时给出检测像素坐标、`world_xyz` 和目标坐标系下的 `target_xyz`，方便和真值对比：

```bash
rostopic echo -n 1 /scene/block_truth
rostopic echo -n 1 /grasp_targets/details
```

常用参数：

```bash
roslaunch ur5_notebook grasp_pose_converter.launch \
  detections_topic:=/detection/results \
  target_frame:=base_link \
  stable_only:=true \
  min_confidence:=0.50 \
  grasp_z_offset:=0.015 \
  approach_z_offset:=0.10
```

### 11. 传统方法对比与 100 次离线验证

为满足 YOLO 与传统算法对比，仓库提供 `traditional_detection.py`。它和 YOLO 订阅同一个腕部相机图像，输出同样的 `Detection2DArray` 结构，并额外发布每帧耗时。

颜色方法：

```bash
roslaunch ur5_notebook traditional_detection.launch method:=color
```

轮廓方法：

```bash
roslaunch ur5_notebook traditional_detection.launch method:=contour
```

模板匹配方法：

```bash
roslaunch ur5_notebook traditional_detection.launch method:=template
```

混合传统方法：

```bash
roslaunch ur5_notebook traditional_detection.launch method:=hybrid
```

查看传统检测输出：

```bash
rostopic echo /traditional_detection/results
rostopic echo /traditional_detection/stats
rqt_image_view /traditional_detection/image_annotated
```

离线评估脚本会从验证集抽取图片，对传统方法和 YOLOv8 同时评估，并生成 `metrics.json`、`per_frame.csv`、`summary.png` 和示例标注图：

```bash
python scripts/evaluate_detection_methods.py \
  --limit 100 \
  --model ur5_ROS-Gazebo/yolo/weights/best.pt \
  --output-dir ur5_ROS-Gazebo/experiments/eval_100
```

输出目录：

```bash
/Users/yingziyu009/Desktop/ur5_grasp_in_gazebo_v1/ur5_ROS-Gazebo/experiments/eval_100
```

当前策略建议固定同一组验证集，重点记录 0-9 的类别准确率、漏检、误检、平均中心点误差和平均耗时。实验结果图可以用于汇报中展示 YOLO 与传统方法的对比。

### 12. 查看场景真值

环境节点会发布生成时的方块和格子位姿，便于调试和统计视觉定位误差：

```bash
rostopic echo /scene/block_truth
rostopic echo /scene/grid_truth
```

也可以查看 ROS 参数：

```bash
rosparam get /scene/block_0_pose
rosparam get /scene/target_1_pose
```

注意：`/scene/block_truth` 和 `/scene/block_X_pose` 记录的是生成时初始位置。如果在 Gazebo GUI 中手动拖动方块，后续定位应以相机实时检测结果为准。

## 场景配置

配置文件：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/config/assignment_scene.yaml
```

常用参数含义：

- `table_x/table_y/table_z`：桌面中心位置。
- `table_size_x/table_size_y/table_size_z`：桌面尺寸。
- `block_size_x/block_size_y/block_size_z`：数字方块尺寸。
- `block_mass`：方块质量。
- `block_mu`：方块摩擦系数。
- `block_spacing`：随机生成时方块中心的最小间距。
- `spawn_x_min/spawn_x_max/spawn_y_min/spawn_y_max`：方块随机生成区域。
- `grid_origin_x/grid_origin_y`：1 号格子中心位置。
- `grid_spacing_x/grid_spacing_y`：相邻格子中心间距。
- `grid_cols/grid_rows`：格子排布，当前为 3 列 2 行。
- `grid_cell_size`：每个格子的边长。
- `randomize`：是否随机生成方块位置。
- `seed`：随机种子，`0` 表示使用当前时间。
- `spawn_obstacles`：是否生成可选障碍物。

修改配置文件后，重新启动 `assignment_env.launch` 生效。

## 13. 抓取执行（Grasp Executor）

抓取流程使用 `gazebo_ros_link_attacher`（已写入 Dockerfile）在物理层把方块焊到 `wrist_3_link`，避免裸 vacuum plugin 的吸附不稳定问题。

**状态机**

```
idle → approaching → descending → contacting → vacuum_on
                                                   ↓
done ← returning_home ← placing ← retreating
```

**执行流（4 个终端，每个先 `docker exec -it ur5_grasp_kinetic bash`）**

```bash
# 终端 1：仿真（保持运行别 Ctrl+C，等到出现 "Assignment scene ready:")
pkill -9 -f gazebo; pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f roslaunch; pkill -9 -f rosmaster; pkill -9 -f move_group; pkill -9 -f robot_state_publisher; pkill -9 -f controller_spawner; pkill -9 -f grasp_executor; pkill -9 -f detection; pkill -9 -f localization; sleep 5
roslaunch ur5_notebook assignment_env.launch seed:=42 gui:=true

# 终端 2：检测+定位
roslaunch ur5_notebook detection_localization.launch

# 终端 3：抓取执行器
roslaunch ur5_notebook grasp_executor.launch

# 终端 4：发指令（class_id=-1 自动选 1~6 中可达的；指定 1~6 抓特定类）
rostopic pub -1 /grasp/target_class_id std_msgs/Int32 "data: -1"
rostopic echo /grasp/status
```

**Demo**

| Demo 1 | Demo 2 | Demo 3 |
|---|---|---|
| ![demo1](demos/demo1.gif) | ![demo2](demos/demo2.gif) | ![demo3](demos/demo3.gif) |

## 14. Task Manager Demo（E 模块：可达性 + 调度）

`task_manager`（E 模块）是任务编排层：读任务表 → 做可达性预检 → 向 `grasp_executor` 下发 `/grasp/target_class_id` → 等待 `/grasp/status`，并订阅 `/localization/grasp_targets` 获取稳定检测与位姿。

```text
/localization/grasp_targets  ──►  task_manager  ──►  /grasp/target_class_id
                                        ▲                    │
                                        │                    ▼
                                   /grasp/status  ◄──  grasp_executor
```

规则：`pick_digit` 必须等于 `place_grid`（digit N 只能放到 grid N）。拓展 1 按 `order` 顺序执行；拓展 7 在**当前可执行候选**里按分数重排。

更完整的交付说明见：`ur5_ROS-Gazebo/docs/TASK_MANAGER_DELIVERY.md`。

### 14.1 四终端启动（推荐 `seed:=42`）

以下命令在**宿主机 PowerShell**执行（路径为容器内绝对路径，不要用 `$(find ...)`）。

**终端 A — 仿真 + VNC**

```powershell
docker exec -it ur5_grasp_kinetic bash -lc "GAZEBO_SEED=42 GAZEBO_GUI=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh"
```

等到日志出现 `Assignment scene ready`（或 Gazebo 稳定约 2 分钟）。  
注意：用 `GAZEBO_SEED=42` 即可；不要设 `GAZEBO_RANDOMIZE=false`（会固定布局、忽略 seed）。

**终端 B — 检测 + 定位**

```powershell
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook detection_localization.launch"
```

**终端 C — 抓取执行器**

```powershell
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook grasp_executor.launch reach_radius_xy:=0.92"
```

等到 `grasp_executor ready`。**C 与 D 的 `reach_radius_xy` 应一致**（见 14.4）。

**终端 D — 任务管理器**（见 14.2 / 14.3）

运行期间 **A/B/C 全程保持，不要 Ctrl+C**。D 会等每步 `done` 后再发下一步（默认间隔 5s）。

若已在容器内 bash，A 可改为：`roslaunch ur5_notebook assignment_env.launch seed:=42 gui:=true`。

### 14.2 Demo A：顺序连抓（Extension 1）

| 任务表 | 说明 |
|--------|------|
| `config/task_table_demo_sequence.yaml` | 可自定义多步顺序（当前示例含 2/6/5） |
| `config/task_table_demo_sequence_5_6.yaml` | 另备序列示例 |

**稳定录屏推荐**：顺序 **6 → 5**（第一步手臂最干净时抓较远的 6，再抓 5）。可在 YAML 里只保留两行并设 `order: 1/2`。含 **digit 2 → grid 2** 时机械臂会去上排边缘，路径长、偶发 Gazebo detach 后 segfault；三步连抓比两步更容易崩。

**终端 D（序列模式 + skip）**

```powershell
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook task_manager.launch task_table_yaml:=/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/config/task_table_demo_sequence.yaml enable_extension7_scheduler:=false unreachable_policy:=skip reach_radius_xy:=0.97 inter_grasp_delay:=5.0"
```

若用 **6→5 两步稳定 demo**，把 `reach_radius_xy` 改为 `0.92`，C 终端 launch 也改为 `reach_radius_xy:=0.92`。

**预期日志**

- `Executing step: step_idx=0 pick_digit=...`
- 每步 `Step finished`；步骤间 `inter_grasp_delay: waiting 5.0s`
- 不可执行步骤输出 JSON `event: skipped`（含 `reason_code`），继续后续步骤
- 全部结束：`All steps done.` 或 `No executable steps left. skipped=N`

### 14.3 Demo B：优先级 + 距离调度（Extension 7）

任务表：`config/task_table_demo_scheduler.yaml`（6 / 2 / 5 三项；**2 放置到上排 grid 2 在默认半径下不可达**，用于演示 skip）。

**终端 D（调度模式）**

```powershell
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook task_manager.launch task_table_yaml:=/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/config/task_table_demo_scheduler.yaml enable_extension7_scheduler:=true alpha:=0.6 unreachable_policy:=skip reach_radius_xy:=0.92 inter_grasp_delay:=5.0"
```

调度公式（仅在当前 stable 且预检通过的候选里排序）：

```text
score = priority - alpha * distance3D
distance3D = sqrt(x² + y² + z²)    # pose_base
```

**预期**：执行顺序可能与 YAML 里 `order` 不同；digit 2 常见 `place_unreachable` → `skipped`；其余如 5→6 正常抓放。

### 14.4 关键参数

| 参数 | 默认 | Demo 建议 | 说明 |
|------|------|-----------|------|
| `unreachable_policy` | `fail` | `skip` | `skip` 跳过不可达/缺 stable 步骤；`fail` 任一步失败即停 |
| `reach_radius_xy` | `0.80` | A: `0.97`（含 2）/ `0.92`（6→5）；B: `0.92` | base 系 `hypot(x,y)` 抓取与放置预检半径；**C 与 D 需一致** |
| `inter_grasp_delay` | `5.0` | `5.0` | 连抓步骤间隔，给 Gazebo 控制器收敛 |
| `enable_extension7_scheduler` | `false` | A: `false`；B: `true` | 是否启用调度 |
| `alpha` | `0.6` | `0.6` | 调度距离权重 |

**`reach_radius_xy` 与 skip 的关系**（`spawn_offset_y=0.7`，seed 42 典型值）：

| 半径 | 效果 |
|------|------|
| `0.92` | grid 2 **放置**预检失败 → skip 2；6/5 下排一般可执行 |
| `0.97` | grid 2 放置预检通过，可抓放 2，但运动大、稳定性差 |

调小 `reach_radius_xy` 可**故意触发 skip** 做演示；调大则放行边缘工位，路径更长。

### 14.5 故障排查

**VNC / Gazebo 打不开或 gzclient 崩溃**

进程未杀干净或 Xvfb 残留。在新终端执行：

```powershell
docker exec ur5_grasp_kinetic bash -c "killall -9 gzserver gzclient rosmaster roslaunch move_group Xvfb x11vnc fluxbox 2>/dev/null; sleep 2; echo cleanup_done"
```

仍异常则：`docker restart ur5_grasp_kinetic`，再按 A→B→C→D 顺序重启。  
不要用 `pkill -9 -f roslaunch` 写在含 `roslaunch` 的同一行命令里（会误杀自身）。

**D 报 `timeout waiting /grasp/status done`**

- C 终端被 Ctrl+C 或未启动
- Gazebo `gzserver` 已 segfault（连抓多次 detach 后偶发）；看 A 终端是否有 `Segmentation fault`
- 处理：重启 A/C，再跑 D；不要提前 Ctrl+C D

**第二步抓放失败**

连抓时第二步目标更远、控制器未收敛。加大 `inter_grasp_delay`，或任务表改为 **先远后近（6→5）**。

**详细联调记录**

见 `ur5_ROS-Gazebo/docs/TASK_MANAGER_DELIVERY.md` 第 11 节。