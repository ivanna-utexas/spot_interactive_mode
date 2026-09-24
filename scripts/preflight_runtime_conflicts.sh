#!/usr/bin/env bash
# preflight_runtime_conflicts.sh — Detect runtime conflicts before launching.
# Checks for: stale navstack processes, duplicate Velodyne publishers,
#              Azure Kinect leftovers, wrong DDS env, bond library conflicts.
# Usage: ./scripts/preflight_runtime_conflicts.sh
# Returns 0 if clean, 1 if conflicts found.
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[0;33m'; RST='\033[0m'
FAIL=0

pass() { printf "${GRN}[OK]${RST}   %s\n" "$1"; }
warn() { printf "${YEL}[WARN]${RST} %s\n" "$1"; }
fail() { printf "${RED}[FAIL]${RST} %s\n" "$1"; FAIL=1; }

echo "===== Preflight Runtime Conflict Check ====="

# ---------- 1. Stale navstack / nav2 processes ----------
NAV2_PROCS=$(pgrep -a -f '(nav2_|lifecycle_manager|controller_server|planner_server|bt_navigator|amcl)' 2>/dev/null || true)
if [[ -n "$NAV2_PROCS" ]]; then
    fail "Stale nav2 processes found:"
    echo "$NAV2_PROCS" | sed 's/^/       /'
    echo "       Run: killros2"
else
    pass "No stale nav2 processes"
fi

# ---------- 2. Duplicate Velodyne publishers ----------
if command -v ros2 &>/dev/null; then
    VEL_PUBS=$(ros2 topic info /velodyne_points --verbose 2>/dev/null | grep -c "Publisher" || true)
    VEL_PUBS="${VEL_PUBS:-0}"
    if [[ "$VEL_PUBS" -gt 1 ]]; then
        fail "Multiple publishers on /velodyne_points ($VEL_PUBS found)"
    elif [[ "$VEL_PUBS" -eq 1 ]]; then
        warn "/velodyne_points already has a publisher (ensure it's intended)"
    else
        pass "No active /velodyne_points publishers"
    fi

    # Also check for duplicate velodyne_driver_node processes
    VEL_DRIVER=$(pgrep -c -f 'velodyne_driver_node' 2>/dev/null || true)
    VEL_DRIVER="${VEL_DRIVER:-0}"
    if [[ "$VEL_DRIVER" -gt 1 ]]; then
        fail "Multiple velodyne_driver_node processes ($VEL_DRIVER)"
    elif [[ "$VEL_DRIVER" -eq 1 ]]; then
        warn "velodyne_driver_node already running"
    else
        pass "No velodyne_driver_node running"
    fi
else
    warn "ros2 CLI not available — skipping topic checks"
fi

# ---------- 3. Azure Kinect leftovers ----------
AK_PROCS=$(pgrep -a -f '(k4a|azure_kinect|depth_engine)' 2>/dev/null || true)
if [[ -n "$AK_PROCS" ]]; then
    warn "Azure Kinect processes detected (may consume USB bandwidth):"
    echo "$AK_PROCS" | sed 's/^/       /'
else
    pass "No Azure Kinect processes"
fi

# ---------- 4. DDS environment ----------
if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]]; then
    fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset} (expected rmw_cyclonedds_cpp)"
else
    pass "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
fi

if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f "${CYCLONEDDS_URI}" ]]; then
        pass "CYCLONEDDS_URI points to existing file"
    else
        fail "CYCLONEDDS_URI=${CYCLONEDDS_URI} — file not found"
    fi
else
    fail "CYCLONEDDS_URI is unset"
fi

# Check for FastDDS env vars that might interfere
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    warn "FASTRTPS_DEFAULT_PROFILES_FILE is set — may interfere with CycloneDDS"
fi
if [[ -n "${ROS_DISCOVERY_SERVER:-}" ]]; then
    warn "ROS_DISCOVERY_SERVER is set — not expected for CycloneDDS"
fi

# ---------- 5. Bond library conflict ----------
WS_ROOT="${HOME}/spot_companion_mode"
BOND_CONFLICT=0
for pkg_dir in bond bondcpp bond_core test_bond smclib; do
    pkg_path="${WS_ROOT}/src/bond_core/${pkg_dir}"
    if [[ -d "$pkg_path" && ! -f "$pkg_path/COLCON_IGNORE" ]]; then
        fail "src/bond_core/${pkg_dir}/ missing COLCON_IGNORE — will override system bond and crash nav2"
        BOND_CONFLICT=1
    fi
done
if [[ $BOND_CONFLICT -eq 0 ]]; then
    pass "bond_core COLCON_IGNORE files in place"
fi

# ---------- 6. Disk space ----------
AVAIL_GB=$(df -BG --output=avail "${WS_ROOT}" 2>/dev/null | tail -1 | tr -d ' G')
if [[ "${AVAIL_GB:-0}" -lt 20 ]]; then
    fail "Low disk space: ${AVAIL_GB}GB available (need ≥20GB for bags)"
elif [[ "${AVAIL_GB:-0}" -lt 50 ]]; then
    warn "Disk space: ${AVAIL_GB}GB available (bags can be large)"
else
    pass "Disk space: ${AVAIL_GB}GB available"
fi

echo ""
if [[ $FAIL -eq 0 ]]; then
    echo -e "${GRN}Preflight clean — safe to launch.${RST}"
    exit 0
else
    echo -e "${RED}Preflight found conflicts. Fix before launching.${RST}"
    exit 1
fi
