#!/usr/bin/env bash
# record_pedestrian_video.sh — Record Azure Kinect video + pedestrian tracking output.
#
# Records the compressed RGB stream (via the image_transport republish node,
# since the k4a driver's own /rgb/image_raw/compressed topic publishes nothing)
# alongside the canonical CenterPoint outputs for replay and evaluation.
#
# Usage: ./scripts/record_pedestrian_video.sh [--name NAME] [--with-lidar]
set -euo pipefail

WS_ROOT="${HOME}/spot_companion_mode"
NAME=""
WITH_LIDAR=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --name) NAME="$2"; shift 2 ;;
        --with-lidar) WITH_LIDAR=1; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

DATE=$(date +%Y-%m-%d)
if [[ -n "$NAME" ]]; then
    RUN_DIR="bags/pedestrian/${DATE}_${NAME}"
else
    RUN_DIR="bags/pedestrian/${DATE}_$(date +%H%M%S)"
fi
BAG_PATH="${WS_ROOT}/${RUN_DIR}"

TOPICS=(
    /rgb/video/compressed
    /rgb/camera_info
    /people_detections
    /people/map_tracks
    /nearby_people
    /people_detections_markers
    /centerpoint_people/diagnostics
    /tf
    /tf_static
)
if [[ "$WITH_LIDAR" -eq 1 ]]; then
    TOPICS+=(/velodyne_points)
fi

echo "===== Pedestrian Tracking Video Recording ====="
echo "Output: $BAG_PATH"
echo "Topics: ${TOPICS[*]}"
echo "Press Ctrl+C to stop."
echo ""

ros2 bag record "${TOPICS[@]}" -o "$BAG_PATH" -s sqlite3

echo ""
echo "Recording saved to: $BAG_PATH"
