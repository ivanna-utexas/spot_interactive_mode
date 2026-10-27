# =============================================================================
# Base Image and Build Arguments
#  - Robot: Jetson AGX Thor (JetPack 7 / L4T R38, Ubuntu 24.04, CUDA 13, sm_110)
#    - There is no l4t-jetpack image for JetPack 7, so the robot uses the arm64
#      (SBSA) CUDA image; the host driver is injected by --runtime nvidia.
#    - Image list: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda/tags
#  - ROS 2 Jazzy, which is the ROS 2 LTS release for Ubuntu 24.04
#  - For native builds: ARM64/Jetson uses the CUDA image, AMD64 uses Ubuntu 24.04
#  - Jetson Orin (JetPack 6 / Humble) is still built from the main branch
# =============================================================================

# Build arguments
ARG ros_distro=jazzy
ARG gid
ARG uid
ARG username
ARG i2c_gid=115
ARG install_cuda=false
ARG is_jetson=false
ARG BASE_STAGE_NAME=base_amd64

# =============================================================================
# Stage 1: Platform-Specific Base Images
# =============================================================================
# For Robot (ARM64/Jetson Thor): CUDA 13 + cuDNN 9 image, TensorRT added in Stage 3
# Note: Stage name uses "arm64" to match Docker's TARGETARCH variable
ARG BASE_IMAGE_ARM64=nvcr.io/nvidia/cuda:13.0.2-cudnn-devel-ubuntu24.04
FROM ${BASE_IMAGE_ARM64} AS base_arm64

ENV CUDA_HOME=/usr/local/cuda
# graphics is needed for EGL/OpenGL (RViz, eye display) and by the Azure Kinect
# depth engine, which unprojects on the GPU. The CUDA image only enables compute.
ENV NVIDIA_DRIVER_CAPABILITIES=all

# For Desktop (AMD64/x86_64): Use Ubuntu 24.04 with optional CUDA
# Note: Stage name uses "amd64" to match Docker's TARGETARCH variable
FROM ubuntu:24.04 AS base_amd64

ARG install_cuda

# Install CUDA toolkit for Desktop only if install_cuda is true
# This provides GPU support on Desktop machines with NVIDIA GPUs
COPY scripts/detect_gpu_setup.sh /tmp/detect_gpu_setup.sh
RUN if [ "$install_cuda" = "true" ]; then \
        . /tmp/detect_gpu_setup.sh && \
        apt-get update && apt-get install -y \
            wget \
            gnupg2 \
            && rm -rf /var/lib/apt/lists/* && \
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
        dpkg -i cuda-keyring_1.1-1_all.deb && \
        rm cuda-keyring_1.1-1_all.deb && \
        apt-get update && apt-get install -y \
            ${CUDA_TOOLKIT_PKG} \
            ${CUDNN_PKG} \
            && rm -rf /var/lib/apt/lists/*; \
    fi
ENV CUDA_HOME=/usr/local/cuda

# =============================================================================
# Stage 2: Select Base - Auto-detects platform during native build
# =============================================================================
# When building natively on Robot (ARM64), this will pull from base-arm64
# When building natively on Desktop (AMD64), this will pull from base-amd64
ARG BASE_STAGE_NAME
FROM $BASE_STAGE_NAME AS base-selected

ARG ros_distro=jazzy
ARG gid
ARG uid
ARG username

# =============================================================================
# Stage 3: Base Setup - Locale, Essential Tools and TensorRT
# =============================================================================
FROM base-selected AS base-setup

# Set non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Ubuntu 24.04 marks the system Python as externally managed (PEP 668), which
# makes every `pip install` below fail. The image is single-purpose, and ROS
# and colcon only see the system interpreter, so install into it on purpose.
# The constraints keep pip from replacing the NumPy that cv_bridge and the
# other Ubuntu/ROS Python packages were built against, and from installing a
# setuptools that colcon-core rejects.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt
RUN printf 'numpy==1.26.4\nsetuptools<80\n' > /etc/pip-constraints.txt

# Setup Locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    iproute2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# TensorRT (people_detector / CUDA-CenterPoint), from the CUDA apt repo that the
# CUDA base images configure. 10.13 is the first release with Thor (sm_110)
# support. Every package is pinned: apt resolves unpinned dependencies to the
# newest TensorRT in the repo (11.x). libnvinfer-bin provides
# /usr/src/tensorrt/bin/trtexec for scripts/generate_centerpoint_engine.sh.
ARG TRT_VERSION=10.13.3.9-1+cuda13.0
RUN if [ -d /usr/local/cuda ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            libnvinfer10=${TRT_VERSION} \
            libnvinfer-lean10=${TRT_VERSION} \
            libnvinfer-dispatch10=${TRT_VERSION} \
            libnvinfer-plugin10=${TRT_VERSION} \
            libnvinfer-vc-plugin10=${TRT_VERSION} \
            libnvonnxparsers10=${TRT_VERSION} \
            libnvinfer-headers-dev=${TRT_VERSION} \
            libnvinfer-headers-plugin-dev=${TRT_VERSION} \
            libnvinfer-dev=${TRT_VERSION} \
            libnvinfer-plugin-dev=${TRT_VERSION} \
            libnvonnxparsers-dev=${TRT_VERSION} \
            libnvinfer-bin=${TRT_VERSION} \
        && apt-mark hold 'libnvinfer*' 'libnvonnxparsers*' \
        && ln -sf /usr/src/tensorrt/bin/trtexec /usr/local/bin/trtexec \
        && rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping TensorRT installation (CUDA not found)"; \
    fi

# CUDA-CenterPoint's bundled ONNX parser sources were generated with protobuf
# 3.21. Keep this ABI-stable toolchain separate from Ubuntu/ROS protobuf.
ARG PROTOBUF_RELEASE=21.12
RUN wget -q "https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOBUF_RELEASE}/protobuf-all-${PROTOBUF_RELEASE}.tar.gz" \
      -O /tmp/protobuf.tar.gz && \
    mkdir -p /tmp/protobuf-src /tmp/protobuf-build && \
    tar -xzf /tmp/protobuf.tar.gz -C /tmp/protobuf-src --strip-components=1 && \
    cmake -S /tmp/protobuf-src/cmake -B /tmp/protobuf-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/protobuf-3.21.12 \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -Dprotobuf_BUILD_TESTS=OFF \
      -Dprotobuf_BUILD_SHARED_LIBS=OFF && \
    cmake --build /tmp/protobuf-build --parallel "$(nproc)" && \
    cmake --install /tmp/protobuf-build && \
    rm -rf /tmp/protobuf.tar.gz /tmp/protobuf-src /tmp/protobuf-build

# =============================================================================
# Stage 4: ROS2 Installation
# =============================================================================
FROM base-setup AS ros2-install

ARG ros_distro
ARG is_jetson

# Setup Official ROS2 APT repository. The ros2-apt-source package installs the
# signing key and keeps it current (the old ros.key download has expired).
ARG ROS_APT_SOURCE_VERSION=1.3.0
RUN curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb

# Install the supported ROS desktop set without the simulator metapackages.
# desktop-full pulls Gazebo into the Jetson image even though this robot
# workspace does not use it.
RUN apt-get update && apt-get install -y \
    ros-${ros_distro}-desktop \
    ros-${ros_distro}-rmw-cyclonedds-cpp \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# =============================================================================
# Stage 5: Hardware Dependencies (Cameras, Sensors)
# =============================================================================
FROM ros2-install AS hardware-deps

# Re-declare build arguments for this stage
ARG ros_distro
ARG is_jetson

# RealSense SDK + ROS wrapper from the ROS apt repo (RSUSB backend: no kernel
# patches, and no source build against CUDA 13)
RUN apt-get update && apt-get install -y \
    ros-${ros_distro}-librealsense2 \
    ros-${ros_distro}-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Azure Kinect SDK
COPY scripts/install_azure_kinect_sdk.sh /tmp/install_azure_kinect_sdk.sh
RUN /tmp/install_azure_kinect_sdk.sh && rm /tmp/install_azure_kinect_sdk.sh

# MPU6050 Dependencies
RUN if [ "$is_jetson" = "true" ]; then \
  apt-get update && apt-get install -y \
  libi2c-dev \
  && rm -rf /var/lib/apt/lists/*; \
fi

# =============================================================================
# Stage 6: Application Dependencies
# =============================================================================
FROM hardware-deps AS app-deps

# Audio capture dependencies (for spot_audio: Azure Kinect mic array)
RUN apt-get update && apt-get install -y \
    libportaudio2 \
    portaudio19-dev \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*


# =============================================================================
# VectorNav VN-100/200/300 ROS 2 driver (build from source)
# =============================================================================
ENV VN_WS=/opt/vn_ws

# Clone ROS 2 branch, resolve deps, build
RUN set -e \
 && mkdir -p "$VN_WS/src" \
 && cd "$VN_WS/src" \
 && git clone -b ros2 --depth 1 https://github.com/dawonn/vectornav.git \
 && cd "$VN_WS" \
 && . /opt/ros/${ros_distro}/setup.sh \
 && apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y -r \
 && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf "$VN_WS/build" "$VN_WS/log" /var/lib/apt/lists/*

# Source the overlay in login + interactive shells
RUN printf '# VectorNav overlay\n[ -f %s/local_setup.bash ] && . %s/local_setup.bash\n' \
           "$VN_WS/install" "$VN_WS/install" \
      | tee /etc/profile.d/10-vectornav.sh >/dev/null \
 && chmod 0644 /etc/profile.d/10-vectornav.sh \
 && printf '\n# VectorNav overlay\n[ -f %s/local_setup.bash ] && . %s/local_setup.bash\n' \
           "$VN_WS/install" "$VN_WS/install" \
      >> /etc/bash.bashrc

# =============================================================================
# Spot Dependencies & OpenCV
# =============================================================================
COPY scripts/install_spot_ros2.sh /tmp/install_spot_ros2.sh
COPY scripts/spot_wrapper_requirements.txt /tmp/spot_wrapper_requirements.txt

# Prefer ROS/Ubuntu OpenCV (matches cv_bridge) and block pip wheels
RUN apt-get update && apt-get install -y \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i -E '/^\s*opencv-(contrib-)?python(-headless)?([<=>].*)?\s*$/d' /tmp/spot_wrapper_requirements.txt

# Run the Spot install script: builds spot-cpp-sdk and bosdyn_msgs from source,
# because their released binaries only target Ubuntu 22.04 / Humble
ENV BOSDYN_WS=/opt/bosdyn_ws
RUN ROS_DISTRO=${ros_distro} /tmp/install_spot_ros2.sh && rm /tmp/install_spot_ros2.sh

# Source the bosdyn_msgs overlay in login + interactive shells
RUN printf '# bosdyn_msgs overlay\n[ -f %s/local_setup.bash ] && . %s/local_setup.bash\n' \
           "$BOSDYN_WS/install" "$BOSDYN_WS/install" \
      | tee /etc/profile.d/10-bosdyn-msgs.sh >/dev/null \
 && chmod 0644 /etc/profile.d/10-bosdyn-msgs.sh \
 && printf '\n# bosdyn_msgs overlay\n[ -f %s/local_setup.bash ] && . %s/local_setup.bash\n' \
           "$BOSDYN_WS/install" "$BOSDYN_WS/install" \
      >> /etc/bash.bashrc

# Safety net: remove any OpenCV wheels that might have been pulled indirectly
RUN python3 -m pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless || true

# Additional ROS2 Packages (Localization, Transport)
RUN apt-get update && apt-get install -y \
    ros-${ros_distro}-controller-interface \
    ros-${ros_distro}-forward-command-controller \
    ros-${ros_distro}-imu-sensor-broadcaster \
    ros-${ros_distro}-robot-localization \
    ros-${ros_distro}-point-cloud-transport \
    ros-${ros_distro}-velodyne \
    ros-${ros_distro}-velodyne-description \
    ros-${ros_distro}-image-transport \
    ros-${ros_distro}-image-transport-plugins \
    ros-${ros_distro}-bondpy \
    ros-${ros_distro}-depth-image-proc \
    ros-${ros_distro}-xacro \
    ros-${ros_distro}-rosbridge-server \
    ros-${ros_distro}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Serial dependencies
RUN apt-get update && apt-get install -y \
    python3-serial libserialport0 \
    && rm -rf /var/lib/apt/lists/*

# GStreamer (for camera streaming)
RUN apt-get update && apt-get install -y \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  && rm -rf /var/lib/apt/lists/*

# Azure Kinect ROS Driver. The humble branch is the newest one; Microsoft no
# longer maintains the driver, so Jazzy build fixes live in docker/patches/.
ARG AZURE_KINECT_ROS_DRIVER_REF=humble
COPY docker/patches/azure_kinect_ros_driver /tmp/azure_kinect_patches
WORKDIR /opt/azure_kinect_ws
RUN apt-get update && \
    mkdir -p src && \
    cd src && \
    git clone -b ${AZURE_KINECT_ROS_DRIVER_REF} https://github.com/microsoft/Azure_Kinect_ROS_Driver.git && \
    for p in /tmp/azure_kinect_patches/*.patch; do \
      [ -e "$p" ] || continue; \
      git -C Azure_Kinect_ROS_Driver apply "$p"; \
    done && \
    cd .. && \
    . /opt/ros/${ros_distro}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log /tmp/azure_kinect_patches /var/lib/apt/lists/*

# Source Azure Kinect Driver in bashrc
RUN echo "source /opt/azure_kinect_ws/install/setup.bash" >> /etc/bash.bashrc

# EGL, OpenGL libraries (for GUI applications)
RUN apt-get update && apt-get install -y \
  libegl1 \
  libegl-mesa0 \
  libgles2 \
  libglvnd-dev \
  libgl1 \
  libglx-mesa0 \
  libgl1-mesa-dri \
  mesa-utils \
  && rm -rf /var/lib/apt/lists/*

# Install Xfce4 and TurboVNC
RUN apt-get update && apt-get install -y \
    xfce4 \
    xfce4-goodies \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then \
      wget https://github.com/TurboVNC/turbovnc/releases/download/3.2.1/turbovnc_3.2.1_amd64.deb -O /tmp/turbovnc.deb; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
      wget https://github.com/TurboVNC/turbovnc/releases/download/3.2.1/turbovnc_3.2.1_arm64.deb -O /tmp/turbovnc.deb; \
    fi && \
    apt-get update && apt-get install -y /tmp/turbovnc.deb && \
    rm -rf /tmp/turbovnc.deb /var/lib/apt/lists/* && \
    ln -s /opt/TurboVNC/bin/* /usr/local/bin/

# Copy VNC startup script
COPY scripts/vnc /usr/local/bin/vnc
RUN chmod +x /usr/local/bin/vnc

# =============================================================================
# Stage 7: Python and ML Dependencies
# =============================================================================
FROM app-deps AS python-ml

ARG install_cuda
ARG is_jetson
# PyTorch CUDA 13 wheels include sm_110 (Thor) and Blackwell desktop GPUs.
# torchaudio is not installed: nothing in src/ uses it.
ARG PT_VER=2.14.1
ARG VIS_VER=0.29.1

# pip
RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-venv \
  python3-setuptools \
  python3-wheel \
  python3-colcon-clean \
  libgfortran5 \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir sounddevice==0.5.1


# pytorch
# if this step fails, run `docker builder prune`
RUN --mount=type=cache,target=/root/.cache/pip \
  if [ "$is_jetson" = "true" ]; then \
    pip3 install torch==${PT_VER} torchvision==${VIS_VER} --index-url https://download.pytorch.org/whl/cu130; \
  elif [ "$install_cuda" = "true" ]; then \
    . /tmp/detect_gpu_setup.sh && \
    pip3 install torch==${PT_VER} torchvision==${VIS_VER} --index-url https://download.pytorch.org/whl/${CUDA_TAG}; \
  else \
    # CPU Fallback
    pip3 install torch==${PT_VER} torchvision==${VIS_VER} --index-url https://download.pytorch.org/whl/cpu; \
  fi

# leg_detector
RUN --mount=type=cache,target=/root/.cache/pip pip3 install pykalman

# =============================================================================
# Stage 8: Navigation and SLAM Libraries
# =============================================================================
FROM python-ml AS nav-slam

ARG ros_distro

# Libraries for graph nav and enml
RUN apt-get update && apt-get install -y \
  libfreeimage3 \
  libfreeimage-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libgtest-dev \
  libeigen3-dev \
  libjpeg8-dev \
  libgoogle-perftools-dev \
  libsuitesparse-dev \
  libatlas-base-dev \
  libblas-dev \
  libopencv-dev \
  liblapack-dev \
  libceres-dev \
  libtbb-dev \
  libncurses-dev \
  libpopt-dev \
  libpcl-dev \
  ros-${ros_distro}-ament-cmake \
  cimg-dev \
  && rm -rf /var/lib/apt/lists/*

# # =============================================================================
# # iceoryx for shared memory communication (optional)
# # =============================================================================
# RUN apt-get update && apt-get install -y \
#     cmake \
#     g++ \
#     git \
#     libacl1-dev \
#     libncurses-dev \
#     pkg-config \
#     && rm -rf /var/lib/apt/lists/*
# WORKDIR /tmp
# RUN git clone https://github.com/eclipse-iceoryx/iceoryx.git
# WORKDIR /tmp/iceoryx
# RUN cmake -Bbuild -Hiceoryx_meta -DBUILD_ALL=ON -DCMAKE_BUILD_TYPE=Release
# RUN cmake --build build --target install -j$(nproc)

# =============================================================================
# Stage 9: Utilities and Additional Scripts
# =============================================================================
FROM nav-slam AS utilities

ARG ros_distro
ARG is_jetson

# Utils
RUN apt-get update && apt-get install -y \
  vim \
  tmux \
  tmuxinator \
  && rm -rf /var/lib/apt/lists/*

# Jetson-stats (ARM64 only, will fail gracefully on AMD64)
RUN --mount=type=cache,target=/root/.cache/pip if [ "$is_jetson" = "true" ]; then \
    pip3 install jetson-stats || echo "jetson-stats not available on this platform"; \
fi

# ROS2 and other dependencies installation scripts
COPY scripts/ros2_deps.sh /tmp/ros2_deps.sh
RUN ROS_DISTRO=${ros_distro} /tmp/ros2_deps.sh && rm /tmp/ros2_deps.sh

# Tracing Deps
RUN apt-get update && apt-get install -y \
    lttng-tools \
    liblttng-ust-dev \
    python3-babeltrace \
    python3-lttng \
    && rm -rf /var/lib/apt/lists/*
# pip can't upgrade packages apt installed (they have no RECORD file), so put
# newer copies of the ones jupyter needs in /usr/local first
RUN pip install --ignore-installed --no-deps tornado psutil lark && \
    pip install \
    jupyter \
    bokeh \
    pandas

# =============================================================================
# Stage 10: Final - User Creation and Configuration
# =============================================================================
FROM utilities AS final

ARG uid
ARG gid
ARG username
ARG ros_distro
ARG i2c_gid

ENV UID=$uid
ENV GID=$gid
ENV USERNAME=$username
ENV ROS_DISTRO=$ros_distro

# Create user and group, then configure sudo
# Ubuntu 24.04 images ship an "ubuntu" user with UID/GID 1000, which the
# existing-user branch below renames to $USERNAME.
RUN set -e && \
    # Create group if it doesn't exist, or rename the one at GID (e.g. "ubuntu")
    if getent group ${GID} >/dev/null 2>&1; then \
        existing_group=$(getent group ${GID} | cut -d: -f1) && \
        if [ "$existing_group" != "${USERNAME}" ] && ! getent group ${USERNAME} >/dev/null 2>&1; then \
            groupmod -n ${USERNAME} $existing_group; \
        fi; \
    else \
        groupadd -g ${GID} ${USERNAME}; \
    fi && \
    # Handle user creation/modification
    if id ${UID} >/dev/null 2>&1; then \
        # User with this UID exists, modify it
        existing_user=$(id -nu ${UID}) && \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $existing_user 2>/dev/null || true && \
        usermod -g ${GID} ${USERNAME} 2>/dev/null || true; \
    else \
        # Create new user
        useradd -u ${UID} -g ${GID} -m -s /bin/bash ${USERNAME}; \
    fi && \
    # Ensure home directory exists and has correct ownership
    mkdir -p /home/${USERNAME} && \
    chown -R ${UID}:${GID} /home/${USERNAME} && \
    # Add to sudo group and configure passwordless sudo
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    visudo -c && \
    # Add the user to the video group for access to video devices
    usermod -aG video ${USERNAME}

# Create groups matching the host's group IDs, handling existing groups
RUN set -e && \
    # Handle i2c group (host GID passed in by ./container as i2c_gid)
    if getent group i2c >/dev/null 2>&1 && getent group ${i2c_gid} >/dev/null 2>&1; then \
        # Both i2c name and the host GID exist - check if they're the same group
        existing_group=$(getent group ${i2c_gid} | cut -d: -f1) && \
        if [ "$existing_group" != "i2c" ]; then \
            # Different groups: delete the one with wrong GID, rename the one at the host GID
            groupdel i2c && \
            groupmod -n i2c $existing_group; \
        fi; \
    elif getent group ${i2c_gid} >/dev/null 2>&1; then \
        # Group with the host GID exists, rename it to i2c
        existing_group=$(getent group ${i2c_gid} | cut -d: -f1) && \
        groupmod -n i2c $existing_group; \
    elif getent group i2c >/dev/null 2>&1; then \
        # i2c group exists with different GID, change its GID to the host GID
        groupmod -g ${i2c_gid} i2c; \
    else \
        # Create new i2c group
        groupadd -g ${i2c_gid} i2c; \
    fi && \
    # Handle dialout group (host GID: 20)
    if getent group dialout >/dev/null 2>&1 && getent group 20 >/dev/null 2>&1; then \
        # Both dialout name and GID 20 exist - check if they're the same group
        existing_group=$(getent group 20 | cut -d: -f1) && \
        if [ "$existing_group" != "dialout" ]; then \
            # Different groups: delete the one with wrong GID, rename the one at 20
            groupdel dialout && \
            groupmod -n dialout $existing_group; \
        fi; \
    elif getent group 20 >/dev/null 2>&1; then \
        # Group with GID 20 exists, rename it to dialout
        existing_group=$(getent group 20 | cut -d: -f1) && \
        groupmod -n dialout $existing_group; \
    elif getent group dialout >/dev/null 2>&1; then \
        # dialout group exists with different GID, change its GID to 20
        groupmod -g 20 dialout; \
    else \
        # Create new dialout group
        groupadd -g 20 dialout; \
    fi && \
    # Handle input group (host GID: 104 or similar, for joystick/gamepad access)
    # Note: The input group GID varies by system, we'll create it if it doesn't exist
    if ! getent group input > /dev/null 2>&1; then \
        groupadd -r input; \
    fi && \
    # Add user to i2c, dialout, and input groups
    usermod -aG i2c,dialout,input,audio ${USERNAME}

# Fix permissions for Azure Kinect workspace
# This allows the non-root user to generate the URDF file at runtime
RUN chown -R ${UID}:${GID} /opt/azure_kinect_ws

# Copy bash_profile files from docker directory
COPY docker/bash_profile.shared /tmp/
COPY docker/bash_profile.robot /tmp/
COPY docker/bash_profile.desktop /tmp/
# Copy setup script
COPY scripts/setup_bash_profile.sh /tmp/
RUN chmod +x /tmp/setup_bash_profile.sh
# Combine shared and platform-specific bash_profile based on architecture
RUN ARCH=$(dpkg --print-architecture) && \
    /tmp/setup_bash_profile.sh ${ARCH} ${USERNAME} ${ROS_DISTRO} ${GID} && \
    rm -f /tmp/bash_profile.shared /tmp/bash_profile.robot /tmp/bash_profile.desktop /tmp/setup_bash_profile.sh

RUN mkdir -p /opt/spot_audio_models && \
    chown -R ${UID}:${GID} /opt/spot_audio_models

# Create .tmux.conf
RUN echo "set -g mouse on" >> /home/${USERNAME}/.tmux.conf && \
    echo "set -g history-limit 10000" >> /home/${USERNAME}/.tmux.conf && \
    chown ${USERNAME}:${GID} /home/${USERNAME}/.tmux.conf

# =============================================================================
# Switch to User Context
# =============================================================================

USER $username
ENV HOME=/home/$username
WORKDIR /home/$username

# Install user level pip dependencies
# NumPy stays at Ubuntu 24.04's 1.26.4 (pinned by /etc/pip-constraints.txt),
# which cv_bridge and the other ROS Python packages were built against.
RUN --mount=type=cache,target=/tmp/pip-cache,mode=0777 PIP_CACHE_DIR=/tmp/pip-cache pip3 install --user \
    onnx \
    onnxruntime \
    flask \
    tensorboard \
    pygame \
    soxr==0.5.0 \
    faster-whisper==1.1.0 \
    ctranslate2==4.5.0
ENV PATH=/home/$username/.local/bin:$PATH

# Pre-download Whisper + Silero VAD models (no runtime network dependency)
# Note: torch.hub.load for silero-vad requires torchaudio (not installed),
# so we download the repo zip and copy the JIT file directly.
RUN mkdir -p /opt/spot_audio_models && \
    python3 -c "from faster_whisper import WhisperModel; WhisperModel('base', download_root='/opt/spot_audio_models/whisper-base', device='cpu', compute_type='int8')" && \
    python3 -c "import urllib.request, zipfile, shutil, os; urllib.request.urlretrieve('https://github.com/snakers4/silero-vad/archive/refs/tags/v5.1.zip', '/tmp/silero_vad.zip'); z = zipfile.ZipFile('/tmp/silero_vad.zip'); jit = [f for f in z.namelist() if f.endswith('silero_vad.jit')][0]; z.extract(jit, '/tmp/silero_extract'); shutil.copy2(os.path.join('/tmp/silero_extract', jit), '/opt/spot_audio_models/silero_vad.jit'); z.close()" && \
    ls -la /opt/spot_audio_models/whisper-base/ && \
    test -f /opt/spot_audio_models/silero_vad.jit && \
    echo "Model provisioning OK"

# =============================================================================
# Setup Unicast
# =============================================================================
COPY scripts/setup_unicast.sh /home/${username}/setup_unicast.sh
RUN /home/${username}/setup_unicast.sh && rm /home/${username}/setup_unicast.sh

# # =============================================================================
# # Superodom dependencies
# # =============================================================================
# WORKDIR /tmp
# RUN git clone http://github.com/strasdat/Sophus.git && \
#       cd Sophus && git checkout 97e7161 && \
#       mkdir build && cd build && \
#       cmake .. -DBUILD_TESTS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
#       make -j8 && sudo make install

# WORKDIR /tmp
# RUN git clone https://github.com/borglab/gtsam.git && \
#       cd gtsam && git checkout 4abef92 && \
#       mkdir build && cd build && \
#       cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
#       make -j8 && sudo make install

# # Livox-SDK2
# WORKDIR /tmp
# RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
#       cd Livox-SDK2 && \
#       mkdir build && cd build && \
#       cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
#       make -j8 && sudo make install
