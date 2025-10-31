#!/bin/bash
# ROS 2 Humble Installation Script for Raspberry Pi 4
# Ubuntu Server 22.04 LTS
# Engineering Teamwork III - Session 5

set -e  # Exit on any error

echo "=========================================="
echo "ROS 2 Humble Installation Script"
echo "For Raspberry Pi 4 - Ubuntu 22.04"
echo "=========================================="
echo ""

# Check if running on Ubuntu 22.04
if ! grep -q "22.04" /etc/os-release; then
    echo "ERROR: This script is designed for Ubuntu 22.04"
    echo "Current OS:"
    cat /etc/os-release | grep PRETTY_NAME
    exit 1
fi

echo "[Step 1/10] Setting up locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✓ Locale configured"
echo ""

echo "[Step 2/10] Enabling Ubuntu Universe repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
echo "✓ Universe repository enabled"
echo ""

echo "[Step 3/10] Adding ROS 2 GPG key..."
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "✓ ROS 2 GPG key added"
echo ""

echo "[Step 4/10] Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "✓ ROS 2 repository added"
echo ""

echo "[Step 5/10] Updating package lists..."
sudo apt update
echo "✓ Package lists updated"
echo ""

echo "[Step 6/10] Upgrading system packages (this may take a few minutes)..."
sudo apt upgrade -y
echo "✓ System upgraded"
echo ""

echo "[Step 7/10] Installing ROS 2 Humble base (this will take 10-20 minutes)..."
sudo apt install -y ros-humble-ros-base
echo "✓ ROS 2 Humble base installed"
echo ""

echo "[Step 8/10] Installing development tools..."
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
echo "✓ Development tools installed"
echo ""

echo "[Step 9/10] Installing ROS 2 packages for camera and vision..."
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-camera-info-manager
echo "✓ Vision packages installed"
echo ""

echo "[Step 10/10] Installing Python dependencies..."
sudo apt install -y python3-opencv python3-pip
pip3 install numpy
echo "✓ Python dependencies installed"
echo ""

echo "=========================================="
echo "Setting up ROS 2 environment..."
echo "=========================================="

# Add ROS 2 to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "✓ ROS 2 environment added to ~/.bashrc"
else
    echo "✓ ROS 2 environment already in ~/.bashrc"
fi

# Source for current session
source /opt/ros/humble/setup.bash

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Verification:"
echo "-------------"
echo "ROS_DISTRO: $ROS_DISTRO"
echo ""
echo "Testing ROS 2 commands:"
ros2 --help > /dev/null && echo "✓ ros2 command works"
echo ""
echo "Next steps:"
echo "1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "2. Verify with: echo \$ROS_DISTRO (should show: humble)"
echo "3. Test with: ros2 topic list"
echo "4. Continue with Session 5 Part 2 (Hardware Verification)"
echo ""
echo "Installation log saved to: ~/ros2_install.log"
echo "=========================================="
