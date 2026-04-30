#!/bin/bash
# Gazebo world file install - place in package/share for ROS2

echo "GhostPilot Gazebo resources installation"
echo "========================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

echo "Package directory: $PKG_DIR"
echo "Worlds: $PKG_DIR/worlds/"
echo "Models: $PKG_DIR/models/"

GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
if [ -z "$GAZEBO_RESOURCE_PATH" ]; then
    export GAZEBO_RESOURCE_PATH="$PKG_DIR/worlds:$PKG_DIR/models:$GAZEBO_RESOURCE_PATH"
else
    export GAZEBO_RESOURCE_PATH="$PKG_DIR/worlds:$PKG_DIR/models:$GAZEBO_RESOURCE_PATH"
fi

echo "GAZEBO_RESOURCE_PATH set to: $GAZEBO_RESOURCE_PATH"
echo "Done."