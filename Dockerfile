FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 AS nvidia_glvnd

FROM osrf/ros:kinetic-desktop-full-xenial

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kinetic


# 基础工具 + ROS 依赖
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    nano \
    lsb-release \
    build-essential \
    cmake \
    python-pip \
    python-dev \
    python-catkin-tools \
    python-rosdep \
    python-rospkg \
    python-numpy \
    python-opencv \
    python-numpy \
    python-scipy \
    python-matplotlib \
    python-opencv \
    mesa-utils \
    x11-utils \
    xvfb \
    x11vnc \
    fluxbox \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    ros-kinetic-moveit \
    ros-kinetic-ur-gazebo \
    ros-kinetic-gazebo-ros \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-controller-manager \
    ros-kinetic-joint-state-controller \
    ros-kinetic-joint-trajectory-controller \
    ros-kinetic-effort-controllers \
    ros-kinetic-position-controllers \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-joint-state-publisher \
    ros-kinetic-xacro \
    ros-kinetic-tf \
    ros-kinetic-tf2-ros \
    ros-kinetic-rviz \
    ros-kinetic-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# The ROS Kinetic Xenial base image ships an old Mesa libGL.  For GPU-enabled
# containers, use NVIDIA's Ubuntu 16.04 GLVND loader so libGL can dispatch to
# the host-mounted NVIDIA driver libraries.
COPY --from=nvidia_glvnd /usr/local/lib/x86_64-linux-gnu/libGL* /usr/local/lib/x86_64-linux-gnu/
COPY --from=nvidia_glvnd /usr/local/lib/x86_64-linux-gnu/libOpenGL* /usr/local/lib/x86_64-linux-gnu/
RUN echo "/usr/local/lib/x86_64-linux-gnu" > /etc/ld.so.conf.d/00-nvidia-glvnd.conf && ldconfig


# 初始化 rosdep；失败不终止，因为老 ROS 源偶尔会有网络/证书问题
RUN rosdep init || true
RUN rosdep update || true

# 创建 catkin 工作区
RUN mkdir -p /root/arm_ws/src

WORKDIR /root/arm_ws

# 每次进入容器自动 source ROS 和工作区
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/arm_ws/devel/setup.bash ]; then source /root/arm_ws/devel/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
