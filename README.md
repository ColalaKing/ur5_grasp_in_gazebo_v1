# UR5 ROS-Gazebo 抓取仿真

本项目实现了 UR5 机械臂在 ROS Kinetic + Gazebo 中的抓取仿真，包含 USB 相机、视觉检测、传送带、方块、MoveIt 运动规划和真空吸盘控制。

## 使用方式

推荐使用 Docker 运行。团队成员拉取当前 GitHub 仓库后，根据自己的电脑条件选择 CPU 版本或 GPU 版本。

- CPU/软件渲染版本：最稳定，适合没有 NVIDIA GPU、Docker GPU 未配置、远程桌面 OpenGL 不稳定的电脑。
- GPU/OpenGL 版本：使用宿主机 NVIDIA GPU 渲染 Gazebo GUI，性能更好，但依赖 NVIDIA 驱动、NVIDIA Container Toolkit 和可用的 X11/GLX 显示环境。

容器内工作区路径统一为：

```bash
/root/arm_ws
```

源码在容器内挂载到：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo
```

## 团队成员首次安装和部署环境

每位成员不需要在宿主机手动安装 ROS Kinetic、Gazebo、MoveIt 等项目依赖，这些依赖已经写入 Docker 镜像。宿主机只需要准备 Git、Docker、Docker Compose，以及对应路线需要的 VNC 或 NVIDIA Docker。

### 1. 安装基础工具

在宿主机执行：

```bash
sudo apt update
sudo apt install -y git ca-certificates curl gnupg
```

检查 Git：

```bash
git --version
```

### 2. 安装 Docker 和 Docker Compose

如果宿主机已经安装 Docker，可以先检查：

```bash
docker --version
docker compose version
```

如果没有 Docker，按 Docker 官方源安装：

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

将当前用户加入 Docker 用户组，避免每次都输入 `sudo docker`：

```bash
sudo usermod -aG docker $USER
newgrp docker
```

重新检查：

```bash
docker --version
docker compose version
docker ps
```

### 3. 进入项目目录


```bash
cd ur5_grasp_in_gazebo_v1
```


### 4. CPU 成员额外安装 VNC 查看器

CPU 路线如果需要 Gazebo GUI，需要在宿主机安装 VNC 客户端：

```bash
sudo apt update
sudo apt install -y tigervnc-viewer
```

检查：

```bash
vncviewer --version
```

### 5. GPU 成员额外配置 NVIDIA Docker

GPU 路线要求宿主机 NVIDIA 驱动正常：

```bash
nvidia-smi
```

然后安装并启用 NVIDIA Container Toolkit：

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

检查 Docker 是否能看到 NVIDIA runtime：

```bash
docker info | grep -i nvidia
```

进入 GPU 容器后还需要用下面命令确认容器内 GPU/OpenGL 正常：

```bash
nvidia-smi
glxinfo -B
```

### 6. 选择运行路线

没有 NVIDIA GPU，或者 GPU Docker/OpenGL 配置不稳定：

```bash
./scripts/start_gazebo_docker.sh
```

有 NVIDIA GPU，并且 `nvidia-smi` 正常：

```bash
./scripts/start_gazebo_docker_gpu.sh
```

进入容器后继续按照下面的 CPU 版本或 GPU 版本步骤运行。

## CPU 版本

### 1. 启动并进入容器

在宿主机仓库根目录执行：

```bash
./scripts/start_gazebo_docker.sh
```

### 2. 编译并加载 ROS 工作区

进入容器后执行：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
rospack profile
rospack find ur5_notebook
```

如果配置正确，最后一条命令应该输出：

```bash
/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo
```

### 3. 命令1：无 GUI 稳定基线

只启动 Gazebo 后端、UR5、MoveIt、传送带和基础场景：

```bash
roslaunch ur5_notebook initialize.launch gui:=false run_app:=false
```

### 4. 命令2：无 GUI 完整抓取程序

```bash
roslaunch ur5_notebook initialize.launch gui:=false run_app:=true
```

### 5. 命令3：CPU + VNC 图形界面

如果 CPU 电脑也需要看到 Gazebo GUI，使用 VNC 方式。先在容器里执行：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
/root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
```

如果要通过 VNC 启动完整抓取程序：

```bash
GAZEBO_RUN_APP=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh
```

然后在宿主机另开一个终端连接输入该命令连接VNC：

```bash
vncviewer localhost:5900
```

备注：如果宿主机没有 `vncviewer`，可以先安装：

```bash
sudo apt update
sudo apt install -y tigervnc-viewer
```

CPU/VNC 路线会强制使用 Mesa/llvmpipe 软件渲染，可以避免 Docker 中常见的 `libGL`、`swrast`、`gzclient` 崩溃和远程桌面 OpenGL 不兼容问题。

## GPU 版本

### 1. 检查宿主机 NVIDIA 驱动

在宿主机执行：

```bash
nvidia-smi
```

如果能正常看到显卡信息，例如 NVIDIA RTX A6000，说明宿主机驱动基本正常。

### 2. 安装并启用 NVIDIA Container Toolkit

在宿主机执行：

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 3. 启动并进入 GPU 容器

在宿主机仓库根目录执行：

```bash
./scripts/start_gazebo_docker_gpu.sh
```

### 4. 检查容器内 GPU 和 OpenGL

进入容器后执行：

```bash
nvidia-smi
glxinfo -B
```

`glxinfo -B` 应该显示 NVIDIA，例如：

```bash
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: NVIDIA RTX A6000/PCIe/SSE2
```

如果这里显示的是 `swrast` 或者提示找不到 RGB GLX visual/fbconfig，说明当前 GPU OpenGL 没接好。

可以尝试重新构建 GPU 容器：

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d --build ur5_grasp
```

### 5. 编译并加载 ROS 工作区

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
catkin_make
source /root/arm_ws/devel/setup.bash
rospack profile
rospack find ur5_notebook
```

### 6. 命令1：GPU GUI 稳定基线

```bash
roslaunch ur5_notebook initialize.launch gui:=true run_app:=false
```

### 7. 命令2：GPU GUI 完整抓取程序

```bash
roslaunch ur5_notebook initialize.launch gui:=true run_app:=true
```

