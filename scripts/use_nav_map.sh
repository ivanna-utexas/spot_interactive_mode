#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${HOME}/spot_companion_mode"
FINAL_DIR="${WS_ROOT}/maps/final"
SRC_NAV_MAP_DIR="${WS_ROOT}/src/spot_nav/spot_nav2/spot_nav/maps"
INSTALL_NAV_MAP_DIR="${WS_ROOT}/install/spot_nav/share/spot_nav/maps"

usage() {
    cat <<'EOF'
Usage:
  scripts/use_nav_map.sh <map-name-or-yaml> [--launch]

Examples:
  scripts/use_nav_map.sh speedway_01
  scripts/use_nav_map.sh ~/spot_companion_mode/maps/final/speedway_01.yaml
  scripts/use_nav_map.sh speedway_01 --launch

What it does:
  1. Selects an existing 2D map (.yaml + .pgm)
  2. Writes ~/spot_companion_mode/maps/final/current_nav.yaml and current_nav.pgm
  3. Copies the selected named map into spot_nav map folders when present

After running this, tmux/navstack will use current_nav.yaml by default.
Pass --launch to immediately start tmux/navstack with the activated map.
EOF
}

rewrite_yaml_image() {
    local source_yaml="$1"
    local image_name="$2"
    local output_yaml="$3"

    awk -v image_name="$image_name" '
        BEGIN { replaced = 0 }
        /^image:/ && !replaced {
            print "image: " image_name
            replaced = 1
            next
        }
        { print }
    ' "$source_yaml" > "$output_yaml"
}

copy_named_map_into_dir() {
    local map_dir="$1"
    local map_name="$2"
    local source_pgm="$3"
    local source_yaml="$4"
    local dest_pgm="${map_dir}/${map_name}.pgm"
    local dest_yaml="${map_dir}/${map_name}.yaml"

    mkdir -p "$map_dir"
    if [ "$source_pgm" != "$dest_pgm" ]; then
        cp "$source_pgm" "$dest_pgm"
    fi

    if [ "$source_yaml" != "$dest_yaml" ]; then
        rewrite_yaml_image "$source_yaml" "${map_name}.pgm" "$dest_yaml"
    fi
}

LAUNCH_NAVSTACK="0"
map_input=""

while [ $# -gt 0 ]; do
    case "$1" in
        --launch)
            LAUNCH_NAVSTACK="1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            if [ -n "${map_input}" ]; then
                echo "Only one map input is supported. Got extra argument: $1" >&2
                exit 1
            fi
            map_input="$1"
            shift
            ;;
    esac
done

if [ -z "${map_input}" ]; then
    usage
    exit 1
fi

if [ -f "$map_input" ]; then
    yaml_path="$map_input"
elif [ -f "${FINAL_DIR}/${map_input}.yaml" ]; then
    yaml_path="${FINAL_DIR}/${map_input}.yaml"
elif [ -f "${SRC_NAV_MAP_DIR}/${map_input}.yaml" ]; then
    yaml_path="${SRC_NAV_MAP_DIR}/${map_input}.yaml"
elif [ -f "${INSTALL_NAV_MAP_DIR}/${map_input}.yaml" ]; then
    yaml_path="${INSTALL_NAV_MAP_DIR}/${map_input}.yaml"
else
    echo "Map not found: ${map_input}" >&2
    echo "Expected either a YAML path or a map name available in maps/final or spot_nav/maps." >&2
    exit 1
fi

if [[ "${yaml_path}" != *.yaml ]]; then
    echo "Expected a .yaml map file, got: ${yaml_path}" >&2
    exit 1
fi

map_name="$(basename "${yaml_path}" .yaml)"
image_field="$(awk -F': *' '/^image:/ {print $2; exit}' "${yaml_path}")"
image_field="${image_field%\"}"
image_field="${image_field#\"}"

if [ -z "${image_field}" ]; then
    echo "Could not find image: entry in ${yaml_path}" >&2
    exit 1
fi

if [[ "${image_field}" = /* ]]; then
    pgm_path="${image_field}"
else
    pgm_path="$(dirname "${yaml_path}")/${image_field}"
fi

if [ ! -f "${pgm_path}" ]; then
    echo "Referenced map image does not exist: ${pgm_path}" >&2
    exit 1
fi

mkdir -p "${FINAL_DIR}"
cp "${pgm_path}" "${FINAL_DIR}/current_nav.pgm"
rewrite_yaml_image "${yaml_path}" "current_nav.pgm" "${FINAL_DIR}/current_nav.yaml"

if [ -d "${SRC_NAV_MAP_DIR}" ]; then
    copy_named_map_into_dir "${SRC_NAV_MAP_DIR}" "${map_name}" "${pgm_path}" "${yaml_path}"
fi

if [ -d "${INSTALL_NAV_MAP_DIR}" ]; then
    copy_named_map_into_dir "${INSTALL_NAV_MAP_DIR}" "${map_name}" "${pgm_path}" "${yaml_path}"
fi

echo "Activated nav map: ${map_name}"
echo "Current nav YAML: ${FINAL_DIR}/current_nav.yaml"
echo
echo "Launch navstack with:"
echo "  cd ~/spot_companion_mode/tmux/navstack && tmuxinator local"
echo
echo "Override per session with:"
echo "  cd ~/spot_companion_mode/tmux/navstack && NAVSTACK_MAP=${map_name} tmuxinator local"

if [ "${LAUNCH_NAVSTACK}" = "1" ]; then
    echo
    echo "Launching tmux/navstack with current_nav..."
    (
        cd "${WS_ROOT}/tmux/navstack"
        tmuxinator local
    )
fi
