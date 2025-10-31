# ROS 2 Humble Installation Commands
## Session 5: ROS 2 Integration - Command Reference

All commands for installing and setting up ROS 2 Humble on Raspberry Pi 4 with Ubuntu Server 22.04 LTS.

---

## Part 1: Install ROS 2 Humble

### Step 1.1: Set System Locale
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 1.2: Enable Ubuntu Universe Repository
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

### Step 1.3: Add ROS 2 Security Key
```bash
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 1.4: Add ROS 2 Package Repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 1.5: Update Package Lists
```bash
sudo apt update
sudo apt upgrade -y
```

### Step 1.6: Install ROS 2 Humble Base
```bash
sudo apt install -y ros-humble-ros-base
```
⏱️ **Wait time:** 10-20 minutes

### Step 1.7: Install Development Tools
```bash
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
```

### Step 1.8: Setup ROS 2 Environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify:**
```bash
echo $ROS_DISTRO
```
Should output: `humble`

### Step 1.9: Install Computer Vision Packages
```bash
sudo apt install -y ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-transport ros-humble-camera-info-manager
```

### Step 1.10: Install Python Dependencies
```bash
sudo apt install -y python3-opencv python3-pip
pip3 install numpy
```

### Step 1.11: Verify ROS 2 Installation
```bash
ros2 topic list
ros2 node list
```

---

## Part 2: Hardware Verification

### Step 2.1: Download Freenove Code
Check if already installed:
```bash
ls ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi
```

If not installed:
```bash
cd ~
git clone --depth 1 https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi
```

### Step 2.2: Quick Motor Test
```bash
cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server
sudo python3 test.py Motor
```

### Step 2.3: Camera Test
```bash
cd ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server
sudo python3 test.py Camera
```

---

## Part 3: Get ROS Code from Git

### Step 3.1: Create Code Directory
```bash
mkdir -p ~/freenove_ros2_code
cd ~/freenove_ros2_code
```

### Step 3.2: Download ROS Node Files
```bash
wget https://raw.githubusercontent.com/RuthraBellan/freenove-ros2-nodes/main/camera_node.py
wget https://raw.githubusercontent.com/RuthraBellan/freenove-ros2-nodes/main/motor_control_node.py
wget https://raw.githubusercontent.com/RuthraBellan/freenove-ros2-nodes/main/lane_follower_node.py
```

**Alternative - Clone entire repository:**
```bash
git clone https://github.com/RuthraBellan/freenove-ros2-nodes.git
cd freenove-ros2-nodes
```

---

## Part 4: Create ROS 2 Workspace

### Step 4.1: Create Workspace Structure
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 4.2: Create ROS 2 Package
```bash
ros2 pkg create freenove_car --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge
```

### Step 4.3: Copy Node Files
```bash
cp ~/freenove_ros2_code/camera_node.py ~/ros2_ws/src/freenove_car/freenove_car/
cp ~/freenove_ros2_code/motor_control_node.py ~/ros2_ws/src/freenove_car/freenove_car/
cp ~/freenove_ros2_code/lane_follower_node.py ~/ros2_ws/src/freenove_car/freenove_car/
```

---

## Part 5: Configure Package

### Step 5.1: Edit setup.py
```bash
cd ~/ros2_ws/src/freenove_car
nano setup.py
```

Add these entry points (inside the `entry_points` dictionary):
```python
'console_scripts': [
    'camera_node = freenove_car.camera_node:main',
    'motor_control_node = freenove_car.motor_control_node:main',
    'lane_follower_node = freenove_car.lane_follower_node:main',
],
```

Save and exit: `Ctrl+O`, `Enter`, `Ctrl+X`

### Step 5.2: Verify package.xml
```bash
nano package.xml
```

Verify these dependencies exist:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv_bridge</depend>
```

---

## Part 6: Build the Workspace

### Step 6.1: Build with Colcon
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 6.2: Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```

**Make it automatic (recommended):**
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 6.3: Verify Package Installation
```bash
ros2 pkg list | grep freenove
```
Should show: `freenove_car`

---

## Part 7: Run & Test System

### Terminal 1: Camera Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car camera_node
```

### Terminal 2: Motor Control Node
```bash
source ~/ros2_ws/install/setup.bash
sudo -E ros2 run freenove_car motor_control_node
```

### Terminal 3: Lane Follower Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car lane_follower_node
```

---

## Part 8: Debugging Commands

### Check Running Nodes
```bash
ros2 node list
```

### Check Active Topics
```bash
ros2 topic list
```

### Monitor Camera Images
```bash
ros2 topic echo /freenove/camera/image_raw --no-arr
```

### Monitor Velocity Commands
```bash
ros2 topic echo /freenove/cmd_vel
```

### Check Topic Information
```bash
ros2 topic info /freenove/camera/image_raw
ros2 topic info /freenove/cmd_vel
```

### Manually Send Velocity Command (Testing)
```bash
ros2 topic pub /freenove/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

## Troubleshooting Commands

### Kill All ROS Nodes
```bash
pkill -9 -f ros2
```

### Check ROS Environment
```bash
printenv | grep ROS
```

### Rebuild Workspace (if needed)
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

### Check GPIO Permissions (for motor control)
```bash
ls -l /dev/gpiomem
```

---

## Quick Reference - Essential Commands

**Source ROS 2:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**List nodes:**
```bash
ros2 node list
```

**List topics:**
```bash
ros2 topic list
```

**Run a node:**
```bash
ros2 run freenove_car <node_name>
```

**Build workspace:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Notes

- Always run motor control with `sudo -E` (needs GPIO access)
- Use `Ctrl+C` to stop nodes
- Source workspace in every new terminal, or add to `~/.bashrc`
- If camera doesn't work, check: `sudo raspi-config` → Interface Options → Camera
