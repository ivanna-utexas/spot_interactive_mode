#!/bin/bash
set -e
set -x

# Azure Kinect SDK (libk4a). Microsoft only publishes it in its Ubuntu 18.04
# repositories (arm64 debs live in the "multiarch" repo), but the 1.4.x debs
# only depend on libraries Ubuntu 24.04 still ships and include the
# proprietary depth engine. k4a-tools (k4aviewer/k4arecorder) is not installed:
# it needs libsoundio1, which 24.04 dropped.

K4A_VERSION="${K4A_VERSION:-1.4.2}"

# Install dependencies
apt-get update && apt-get install -y \
    curl \
    gnupg

# Determine architecture
ARCH=$(dpkg --print-architecture)

if [ "$ARCH" = "amd64" ]; then
    echo "Installing Azure Kinect SDK for AMD64..."
    REPO="deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/ubuntu/18.04/prod bionic main"
elif [ "$ARCH" = "arm64" ]; then
    echo "Installing Azure Kinect SDK for ARM64..."
    REPO="deb [arch=arm64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor -o /usr/share/keyrings/microsoft.gpg
echo "$REPO" > /etc/apt/sources.list.d/microsoft-k4a.list

# Update and Install SDK
apt-get update
# ACCEPT_EULA=Y answers the debconf license prompt in the package's preinst
ACCEPT_EULA=Y DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libk4a1.4=${K4A_VERSION} \
    libk4a1.4-dev=${K4A_VERSION}

# Remove the bionic repository so its packages can't leak into later apt operations
rm /etc/apt/sources.list.d/microsoft-k4a.list
rm -rf /var/lib/apt/lists/*
