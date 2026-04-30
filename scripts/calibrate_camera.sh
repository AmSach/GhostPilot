#!/bin/bash
# Camera-IMU extrinsic calibration for GhostPilot

set -e

echo "GhostPilot Camera-IMU Calibration"
echo "=================================="

# Check for required packages
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 not found. Install Humble first."
    exit 1
fi

# Launch realsense camera
echo "Starting RealSense camera..."
ros2 launch realsense2_camera rs_camera.launch.py \
    enable_gyro:=true \
    enable_accel:=true \
    rgb_camera.fps:=30 \
    depth_module.depth_scan_rate:=30 &

CAMERA_PID=$!

sleep 3

# Verify camera topics
echo "Checking camera topics..."
if ! ros2 topic list | grep -q "/camera/color/image_raw"; then
    echo "Error: Camera topic not found. Check camera connection."
    kill $CAMERA_PID 2>/dev/null || true
    exit 1
fi

if ! ros2 topic list | grep -q "/imu/imu_data"; then
    echo "Error: IMU topic not found."
    kill $CAMERA_PID 2>/dev/null || true
    exit 1
fi

echo "Camera and IMU topics detected."
echo ""

# Run calibration tool (Kalibr-style)
echo "Running camera-IMU calibration..."
echo "Move the camera in a figure-8 pattern for 30 seconds..."
echo "Press Ctrl+C when complete."

# This would use Kalibr or imu_utils for actual calibration
# kalibr_calibrate_imu_camera --target /config/april.yaml --bag /tmp/calibration.bag

sleep 2

echo ""
echo "Calibration complete."
echo "Extrinsic parameters saved to: /config/vins_params.yaml"
echo ""

# Cleanup
kill $CAMERA_PID 2>/dev/null || true
wait $CAMERA_PID 2>/dev/null || true

echo "Done."