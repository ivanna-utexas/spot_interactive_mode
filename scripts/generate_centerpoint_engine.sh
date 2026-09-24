#!/usr/bin/env bash
# Generate a TensorRT-version-specific CenterPoint head engine in runtime storage.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${WS_ROOT}/src/Lidar_AI_Solution/CUDA-CenterPoint/model"
RUNTIME_DIR="${WS_ROOT}/.navws_runtime/centerpoint"
ONNX_PATH="${MODEL_DIR}/rpn_centerhead_sim.onnx"

if [[ ! -f "${ONNX_PATH}" ]]; then
    echo "Missing ${ONNX_PATH}. Initialize src/Lidar_AI_Solution first." >&2
    exit 2
fi

TRTEXEC="${TRTEXEC:-}"
if [[ -z "${TRTEXEC}" ]]; then
    if command -v trtexec >/dev/null 2>&1; then
        TRTEXEC="$(command -v trtexec)"
    elif [[ -x /usr/src/tensorrt/bin/trtexec ]]; then
        TRTEXEC=/usr/src/tensorrt/bin/trtexec
    else
        echo "TensorRT trtexec was not found in the container." >&2
        exit 3
    fi
fi

VERSION_HEADER=""
for candidate in \
    /usr/include/aarch64-linux-gnu/NvInferVersion.h \
    /usr/include/x86_64-linux-gnu/NvInferVersion.h \
    /usr/include/NvInferVersion.h
do
    if [[ -f "${candidate}" ]]; then
        VERSION_HEADER="${candidate}"
        break
    fi
done
if [[ -z "${VERSION_HEADER}" ]]; then
    echo "NvInferVersion.h was not found; TensorRT development files are required." >&2
    exit 4
fi

TRT_MAJOR="$(awk '$2 == "NV_TENSORRT_MAJOR" {print $3}' "${VERSION_HEADER}")"
TRT_MINOR="$(awk '$2 == "NV_TENSORRT_MINOR" {print $3}' "${VERSION_HEADER}")"
TRT_PATCH="$(awk '$2 == "NV_TENSORRT_PATCH" {print $3}' "${VERSION_HEADER}")"
if [[ -z "${TRT_MAJOR}" || -z "${TRT_MINOR}" || -z "${TRT_PATCH}" ]]; then
    echo "Unable to determine TensorRT version from ${VERSION_HEADER}." >&2
    exit 5
fi

mkdir -p "${RUNTIME_DIR}"
ENGINE_NAME="rpn_centerhead_sim.trt${TRT_MAJOR}.${TRT_MINOR}.${TRT_PATCH}.plan"
ENGINE_PATH="${RUNTIME_DIR}/${ENGINE_NAME}"
LOG_PATH="${RUNTIME_DIR}/${ENGINE_NAME}.log"

if [[ ! -s "${ENGINE_PATH}" ]]; then
    echo "Generating ${ENGINE_PATH}; this normally takes several minutes."
    "${TRTEXEC}" \
        --onnx="${ONNX_PATH}" \
        --saveEngine="${ENGINE_PATH}" \
        --memPoolSize=workspace:4096 \
        --fp16 \
        --outputIOFormats=fp16:chw \
        --inputIOFormats=fp16:chw \
        --profilingVerbosity=detailed >"${LOG_PATH}" 2>&1
fi

"${TRTEXEC}" --loadEngine="${ENGINE_PATH}" --skipInference >"${LOG_PATH}.validate" 2>&1 || {
    echo "TensorRT could not deserialize ${ENGINE_PATH}; see ${LOG_PATH}.validate." >&2
    exit 6
}
ln -sfn "${ENGINE_NAME}" "${RUNTIME_DIR}/rpn_centerhead_sim.plan"
echo "CenterPoint engine ready: ${RUNTIME_DIR}/rpn_centerhead_sim.plan -> ${ENGINE_NAME}"
