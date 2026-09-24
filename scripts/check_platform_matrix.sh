#!/usr/bin/env bash
# check_platform_matrix.sh — Verify the Orin/Docker/ROS runtime matches expectations.
# Usage: ./scripts/check_platform_matrix.sh
# Returns 0 if all checks pass, 1 otherwise.
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; RST='\033[0m'
FAIL=0

pass() { printf "${GRN}[PASS]${RST} %s\n" "$1"; }
warn() { printf "${YEL}[WARN]${RST} %s\n" "$1"; }
fail() { printf "${RED}[FAIL]${RST} %s\n" "$1"; FAIL=1; }

echo "===== Platform Matrix Check ====="

# 1. Architecture
ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)
if [[ "$ARCH" == "arm64" || "$ARCH" == "aarch64" ]]; then
    pass "Architecture: $ARCH (Jetson/ARM64)"
else
    warn "Architecture: $ARCH (not ARM64 — desktop build?)"
fi

# 2. Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "${VERSION_ID:-}" == "22.04" ]]; then
        pass "Ubuntu $VERSION_ID (Jammy)"
    else
        fail "Ubuntu ${VERSION_ID:-unknown}, expected 22.04"
    fi
else
    fail "/etc/os-release not found"
fi

# 3. ROS 2 distro
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
    pass "ROS_DISTRO=$ROS_DISTRO"
else
    fail "ROS_DISTRO=${ROS_DISTRO:-unset}, expected humble"
fi

# 4. RMW implementation
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]]; then
    pass "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
    fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}, expected rmw_cyclonedds_cpp"
fi

# 5. CycloneDDS URI exists
if [[ -n "${CYCLONEDDS_URI:-}" && -f "${CYCLONEDDS_URI}" ]]; then
    pass "CYCLONEDDS_URI=$CYCLONEDDS_URI (file exists)"
elif [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    fail "CYCLONEDDS_URI=$CYCLONEDDS_URI (file missing)"
else
    fail "CYCLONEDDS_URI is unset"
fi

# 6. Key ROS packages installed
for pkg in ros-humble-velodyne ros-humble-rmw-cyclonedds-cpp ros-humble-pcl-conversions; do
    if dpkg -l "$pkg" &>/dev/null; then
        pass "APT package: $pkg"
    else
        fail "APT package missing: $pkg"
    fi
done

# 7. GTSAM available
if ldconfig -p 2>/dev/null | grep -q libgtsam; then
    pass "GTSAM shared library found"
else
    warn "GTSAM shared library not in ldconfig (may be in /usr/local)"
fi

# 8. PCL available
if pkg-config --exists pcl_common 2>/dev/null; then
    PCL_VER=$(pkg-config --modversion pcl_common)
    pass "PCL $PCL_VER"
else
    warn "PCL not found via pkg-config"
fi

# 9. CUDA availability (informational)
if command -v nvcc &>/dev/null; then
    CUDA_VER=$(nvcc --version | grep -oP 'release \K[0-9.]+')
    pass "CUDA $CUDA_VER"
elif [ -d /usr/local/cuda ]; then
    pass "CUDA directory exists at /usr/local/cuda"
else
    warn "CUDA not detected (GPU acceleration unavailable)"
fi

# 10. Docker IPC mode (check /proc for shared memory)
if [[ -d /dev/shm ]]; then
    SHM_SIZE=$(df -h /dev/shm 2>/dev/null | awk 'NR==2{print $2}')
    pass "Shared memory available: $SHM_SIZE"
else
    warn "/dev/shm not available (--ipc=host may be missing)"
fi

echo ""
if [[ $FAIL -eq 0 ]]; then
    echo -e "${GRN}All platform checks passed.${RST}"
    exit 0
else
    echo -e "${RED}Some platform checks failed. Review above.${RST}"
    exit 1
fi
