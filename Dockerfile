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

# Ubuntu 16.04's python-opencv is OpenCV 2.4. Add a Python 2 compatible
# OpenCV 4 wheel so the ROS wrapper and traditional baseline share predictable
# image encoding, contour, and drawing behavior.
RUN pip install --no-cache-dir --upgrade pip==20.3.4 setuptools==44.1.1 && \
    pip install --no-cache-dir --only-binary=:all: wheel==0.37.1 opencv-python-headless==4.1.2.30

ENV CONDA_DIR=/opt/miniconda
ENV MINICONDA_VERSION=py310_23.11.0-2
ENV YOLO_PYTHON=/opt/yolo_py/bin/python
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
ENV MPLCONFIGDIR=/tmp/matplotlib

# Keep ROS Kinetic on Python 2, but add an isolated Python 3 CPU inference
# runtime for Ultralytics YOLOv8. The ROS node talks to this runtime through a
# small JSON-lines worker process, avoiding rospy/cv_bridge Python-version
# conflicts on Ubuntu 16.04.
# Pin the installer: current "latest" Miniconda requires GLIBC 2.28, while
# Ubuntu 16.04 in ROS Kinetic provides GLIBC 2.23.
RUN curl -fL --retry 5 --retry-delay 3 \
      "https://repo.anaconda.com/miniconda/Miniconda3-${MINICONDA_VERSION}-Linux-x86_64.sh" \
      -o /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p "$CONDA_DIR" && \
    rm -f /tmp/miniconda.sh && \
    "$CONDA_DIR/bin/conda" create -y -p /opt/yolo_py python=3.10 pip && \
    /opt/yolo_py/bin/python -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    /opt/yolo_py/bin/python -m pip install --no-cache-dir \
      --index-url https://download.pytorch.org/whl/cpu \
      torch==2.2.2 torchvision==0.17.2 && \
    /opt/yolo_py/bin/python -m pip install --no-cache-dir \
      ultralytics==8.4.53 \
      opencv-python-headless==4.10.0.84 \
      numpy==1.26.4 && \
    "$CONDA_DIR/bin/conda" clean -afy && \
    mkdir -p "$YOLO_CONFIG_DIR" "$MPLCONFIGDIR"

RUN /opt/yolo_py/bin/python -m pip install --no-cache-dir albumentations==2.0.8

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
