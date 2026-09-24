#!/bin/bash

CONFIG_FILE_PATH="$HOME/spot_companion_mode/config"

# if the hostname is "spot" or "spot-orin" set the config to spot.xml and if the hostname is "spot-laptop" set the config to laptop.xml
HOSTNAME=$(hostname)
if [[ "$HOSTNAME" == "spot" || "$HOSTNAME" == "spot-orin" ]]; then
    CONFIG_FILE_NAME="cyclonedds_spot.xml"
elif [[ "$HOSTNAME" == "spot-laptop" ]]; then
    CONFIG_FILE_NAME="cyclonedds_laptop.xml"
else
    # Default fallback
    CONFIG_FILE_NAME="cyclonedds.xml"
    echo "Warning: Hostname '$HOSTNAME' not recognized as spot, spot-orin, or spot-laptop. Using default cyclonedds.xml."
fi

# The XML file that configures Cyclone DDS (stored in user's home directory)


# Export the necessary ROS 2 and Cyclone DDS environment variables (idempotent)
if grep -q "^export ROS_VERSION=" ~/.bashrc; then
    echo "Updating ROS_VERSION in ~/.bashrc"
    sed -i 's|^export ROS_VERSION=.*|export ROS_VERSION=2|' ~/.bashrc
else
    echo "Adding ROS_VERSION to ~/.bashrc"
    echo "export ROS_VERSION=2" >> ~/.bashrc
fi

if grep -q "^export ROS_DOMAIN_ID=" ~/.bashrc; then
    echo "Updating ROS_DOMAIN_ID in ~/.bashrc"
    sed -i 's|^export ROS_DOMAIN_ID=.*|export ROS_DOMAIN_ID=7|' ~/.bashrc
else
    echo "Adding ROS_DOMAIN_ID to ~/.bashrc"
    echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc
fi

if grep -q "^export RMW_IMPLEMENTATION=" ~/.bashrc; then
    echo "Updating RMW_IMPLEMENTATION in ~/.bashrc"
    sed -i 's|^export RMW_IMPLEMENTATION=.*|export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp|' ~/.bashrc
else
    echo "Adding RMW_IMPLEMENTATION to ~/.bashrc"
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
fi

if grep -q "^export CYCLONEDDS_URI=" ~/.bashrc; then
    echo "Updating CYCLONEDDS_URI in ~/.bashrc"
    sed -i "s|^export CYCLONEDDS_URI=.*|export CYCLONEDDS_URI=$CONFIG_FILE_PATH/$CONFIG_FILE_NAME|" ~/.bashrc
else
    echo "Adding CYCLONEDDS_URI to ~/.bashrc"
    echo "export CYCLONEDDS_URI=$CONFIG_FILE_PATH/$CONFIG_FILE_NAME" >> ~/.bashrc
fi

sudo chown -R ros:ros /usr/local/zed/resources 2>/dev/null || true
sudo chmod 2775 /usr/local/zed/resources 2>/dev/null || true
sudo find /usr/local/zed/resources -type d -exec chmod 2775 {} + 2>/dev/null || true
sudo find /usr/local/zed/resources -type f -exec chmod 0664 {} + 2>/dev/null || true