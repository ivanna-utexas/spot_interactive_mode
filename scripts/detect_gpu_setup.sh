#!/bin/bash
# Sourced by docker/Dockerfile on Desktop (AMD64) builds with an NVIDIA GPU.
# Selects the CUDA toolkit / cuDNN apt packages and the matching PyTorch wheel
# index. CUDA 13 matches the Thor image and supports Blackwell (RTX 50) GPUs.

CUDA_VERSION_MAJOR_MINOR="${CUDA_VERSION_MAJOR_MINOR:-13.0}"
CUDA_MAJOR="${CUDA_VERSION_MAJOR_MINOR%%.*}"

export CUDA_TOOLKIT_PKG="cuda-toolkit-${CUDA_VERSION_MAJOR_MINOR/./-}"
export CUDNN_PKG="libcudnn9-dev-cuda-${CUDA_MAJOR}"
export CUDA_TAG="cu${CUDA_VERSION_MAJOR_MINOR/./}"
