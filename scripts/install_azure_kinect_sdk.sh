#!/bin/bash
set -e
set -x

# Install dependencies
apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg \
    software-properties-common

# Determine architecture
ARCH=$(dpkg --print-architecture)

if [ "$ARCH" = "amd64" ]; then
    echo "Installing Azure Kinect SDK for AMD64..."
    # Install libsoundio1 (required by libk4a, removed in 22.04)
    wget -q http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb -O /tmp/libsoundio1.deb
    dpkg -i /tmp/libsoundio1.deb

    # Configure Microsoft Repository for AMD64
    curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
    apt-add-repository "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main"

elif [ "$ARCH" = "arm64" ]; then
    echo "Installing Azure Kinect SDK for ARM64..."
    # Install libsoundio1 (required by libk4a, removed in 22.04)
    wget -q http://ports.ubuntu.com/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_arm64.deb -O /tmp/libsoundio1.deb
    dpkg -i /tmp/libsoundio1.deb

    # Configure Microsoft Repository for ARM64
    curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
    apt-add-repository "deb [arch=arm64] https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main"

else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

# Clean up libsoundio file
rm /tmp/libsoundio1.deb

# Update and Install SDK
apt-get update
# ACCEPT_EULA=Y is often needed for the tools, but good practice for the SDK if it prompts
ACCEPT_EULA=Y DEBIAN_FRONTEND=noninteractive apt-get install -y libk4a1.4 libk4a1.4-dev