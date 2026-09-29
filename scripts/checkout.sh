#!/bin/bash
# Clones the external repos this workspace depends on into src/.
# Trimmed from spot_companion_mode/scripts/checkout.sh: the tracing repos
# (ros2_tracing, tracetools_analysis) are dropped because this project
# doesn't use them. Everything cloned here is listed in .gitignore so its
# history never gets committed into this repo.

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(realpath "$THIS_DIR/..")"
SRC_DIR="$PROJECT_ROOT/src"

cd "$SRC_DIR"

function clone_or_pull {
    REPO_BRANCH=$1
    REPO_URL=$2
    REPO_DIR=$3

    if [ -d "$REPO_DIR/.git" ]; then
        echo "Pulling latest changes for $REPO_DIR"
        cd "$REPO_DIR"
        git pull origin "$REPO_BRANCH"
        git submodule update --init --recursive
        cd "$SRC_DIR"
    else
        echo "Cloning $REPO_URL into $REPO_DIR"
        # --recursive: spot_ros2 pulls in spot_wrapper as a submodule. The
        # original script only initialised submodules on pull, not on the
        # first clone.
        git clone --recursive -b "$REPO_BRANCH" "$REPO_URL" "$SRC_DIR/$REPO_DIR"
    fi
}

# Spot ROS 2 driver (also provides spot_msgs)
clone_or_pull main https://github.com/bdaiinstitute/spot_ros2.git spot_ros2

# Lab repo containing spot_joy (PS4 teleop + deadman) and spot_nav2
clone_or_pull master git@github.com:ut-amrl/spot_nav.git spot_nav

# Velodyne bringup on Spot (feeds /velodyne_points to people_detector)
clone_or_pull master git@github.com:ut-amrl/spot_velodyne.git spot_velodyne

# Robot description (URDF / sensor frames)
clone_or_pull master git@github.com:ut-amrl/ironback_description.git ironback_description
