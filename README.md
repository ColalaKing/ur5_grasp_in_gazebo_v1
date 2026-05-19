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

CPU + VNC 会在容器内启动一个虚拟桌面，通过软件渲染显示 Gazebo GUI，适合没有 NVIDIA GPU、远程桌面 OpenGL 不稳定、或者 Docker X11 直连失败的情况。

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
vncviewer localhost:5900
```

VNC 默认启动当前大作业环境，等价于 `assignment_env.launch gui:=true randomize:=true`。如果要修改启动参数，可以在容器内用环境变量控制：

```bash
GAZEBO_RANDOMIZE=false /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_SEED=42 /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_SPAWN_OBSTACLES=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
GAZEBO_PAUSED=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
```

如果端口 `5900` 被占用，可以指定其他端口：

```bash
GAZEBO_VNC_PORT=5901 /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
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

### 4. 查看相机话题

仿真启动后，另开一个容器终端：

```bash
docker exec -it ur5_grasp_kinetic bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
rostopic list | grep -E "usbcam|camera"
```

正常会看到：

```bash
/ur5/usbcam/camera_info
/ur5/usbcam/image_raw
```

查看图像频率：

```bash
rostopic hz /ur5/usbcam/image_raw
```

查看相机参数：

```bash
rostopic echo -n 1 /ur5/usbcam/camera_info
```

如需打开图像窗口：

```bash
rqt_image_view /ur5/usbcam/image_raw
```

注意：当前相机是腕部相机，安装在机械臂末端，初始姿态不一定正对所有方块。

### 5. 查看场景真值

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
