#!/bin/bash
# Installs the Spot SDK pieces spot_ros2 needs that are not in the ROS apt repo.
#
# Boston Dynamics / RAI only release spot-cpp-sdk and bosdyn_msgs binaries for
# Ubuntu 22.04 + Humble (built against protobuf 3.12 / gRPC 1.30), so on
# Ubuntu 24.04 + Jazzy both are built from source against the system
# protobuf / gRPC:
#   - spot-cpp-sdk -> /opt/spot-cpp-sdk (where bosdyn_cmake_module looks for it)
#   - bosdyn_msgs  -> $BOSDYN_WS (a merged colcon install, sourced by the image)
# spot_ros2 itself is cloned into src/ by scripts/checkout.sh; this script only
# installs its rosdep dependencies from the same pinned commit.
set -euo pipefail
set -x

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SDK_VERSION="5.0.1"
MSG_VERSION="${SDK_VERSION}"
# Keep in sync with scripts/checkout.sh
SPOT_ROS2_REF="949feb7b0d702d1923e5b5dd5c0a4a7728ddc88f"
REQUIREMENTS_FILE=/tmp/spot_wrapper_requirements.txt
BOSDYN_WS="${BOSDYN_WS:-/opt/bosdyn_ws}"
JOBS="${JOBS:-$(nproc)}"

apt-get update && apt-get install -y \
    python3-rosdep \
    python3-pip \
    python3-apt \
    wget \
    qttools5-dev \
    libcli11-dev \
    libeigen3-dev \
    libgrpc++-dev \
    libprotobuf-dev \
    protobuf-compiler \
    protobuf-compiler-grpc

if test -f "$REQUIREMENTS_FILE"; then
    # pip can't upgrade packages apt installed (they have no RECORD file), so
    # install newer copies of the ones the requirements need next to them in
    # /usr/local, then resolve the rest normally
    pip3 install --no-cache-dir --ignore-installed --no-deps typing_extensions blinker cryptography
    pip3 install --no-cache-dir -r "$REQUIREMENTS_FILE"
else
    echo "ERROR: $REQUIREMENTS_FILE not found."
    exit 1
fi

# Initialize only if a sources list definition doesn't exist yet - avoids the rosdep error message
if ! compgen -G "/etc/ros/rosdep/sources.list.d/*default.list" >/dev/null; then
  rosdep init
fi
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
rosdep update

# Install spot-cpp-sdk
git clone --depth 1 --branch "v${SDK_VERSION}" https://github.com/rai-opensource/spot-cpp-sdk.git /tmp/spot-cpp-sdk
cmake -S /tmp/spot-cpp-sdk/cpp -B /tmp/spot-cpp-sdk/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/spot-cpp-sdk \
    -DBUILD_CHOREOGRAPHY_LIBS=ON \
    -DCMAKE_CXX_FLAGS="-include cstdint"  # GCC 13 no longer includes <cstdint> transitively
cmake --build /tmp/spot-cpp-sdk/build --parallel "$JOBS"
cmake --install /tmp/spot-cpp-sdk/build
rm -rf /tmp/spot-cpp-sdk ~/.cmake/packages/bosdyn

# Install bosdyn_msgs - automatic conversions of BD protobufs to ROS messages
mkdir -p "$BOSDYN_WS/src"
# Without --recurse-submodules: the proto2ros submodule pinned by 5.0.1 fails on
# Python 3.12, so rosdep installs the fixed ros-${ROS_DISTRO}-proto2ros instead
git clone --depth 1 --branch "${MSG_VERSION}" \
    https://github.com/rai-opensource/bosdyn_msgs.git "$BOSDYN_WS/src/bosdyn_msgs"
# bosdyn-spot-cpp is the spot-cpp-sdk built above; the -pip keys are covered
# by spot_wrapper_requirements.txt
rosdep install --from-paths "$BOSDYN_WS/src" --ignore-src -y --rosdistro "$ROS_DISTRO" \
    --skip-keys "bosdyn-spot-cpp python3-bosdyn-api-pip"
(
  cd "$BOSDYN_WS"
  colcon build --merge-install --parallel-workers "$JOBS" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
)
rm -rf "$BOSDYN_WS/build" "$BOSDYN_WS/log"

# Install spot_ros2's own ROS dependencies
git clone https://github.com/bdaiinstitute/spot_ros2.git /tmp/spot_ros2
git -C /tmp/spot_ros2 checkout "$SPOT_ROS2_REF"
git -C /tmp/spot_ros2 submodule update --init --recursive
# bosdyn* come from the builds above; spot_wrapper is a plain Python package
# that colcon builds from its setup.py
rosdep install --from-paths /tmp/spot_ros2 --ignore-src -y --rosdistro "$ROS_DISTRO" \
    --skip-keys "bosdyn bosdyn_msgs bosdyn_api_msgs bosdyn_cmake_module bosdyn_spot_api_msgs spot_wrapper"
rm -rf /tmp/spot_ros2 /var/lib/apt/lists/*
