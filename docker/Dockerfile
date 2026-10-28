# ============================================================================
# Re4Life ROS2 app image — Raspberry Pi 5 (arm64), ROS2 Jazzy, NO CUDA.
# Depth inference: CPU onnxruntime for plumbing tests; HailoRT is deliberately
# NOT installed yet — it lands in Phase B together with the compiled HEF, so
# its version can be pinned to the HEF compiler version. Until then the
# 'hailo' backend in spatial_projector fails loudly at startup (by design —
# no silent CPU fallback).
# ============================================================================
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    locales curl gnupg2 software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS2 Jazzy repository (arch auto-detects arm64)
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    v4l-utils \
    iproute2 \
    fonts-dejavu-core \
    wget \
    ros-jazzy-v4l2-camera \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-twist-mux \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-web-video-server \
    && rm -rf /var/lib/apt/lists/*
# If ros-jazzy-web-video-server is ever missing from the mirror, drop it here
# and build web_video_server from source in src/ instead.
# iproute2: qr_generator.py shells out to `ip -4 addr show wlan0` for the AP
# IP — without it the generator silently falls back to the default-route IP
# (the ETHERNET address during development), putting the wrong broker_ip in
# the QR. fonts-dejavu-core: the QR header text (cosmetic).

# Python deps:
#  - onnxruntime (CPU, arm64): depth plumbing tests pre-Hailo
#  - paho-mqtt: the mqtt_bridge node
#  - qrcode/pillow/requests/cryptography: qr_generator.py (provisioning service
#    reuses this image)
RUN pip3 install --break-system-packages \
    onnxruntime \
    numpy \
    paho-mqtt \
    requests \
    cryptography \
    qrcode \
    pillow

WORKDIR /robot_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f /robot_ws/install/setup.bash ] && source /robot_ws/install/setup.bash" >> /root/.bashrc

CMD ["sleep", "infinity"]
