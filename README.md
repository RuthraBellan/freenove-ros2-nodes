# Freenove ROS 2 Nodes

**ROS 2 integration for Freenove 4WD Smart Car Kit**

Course: Engineering Teamwork III - AI and Autonomous Systems Lab  
Session 5: ROS 2 Integration

---

## 📋 Overview

This repository contains three ROS 2 nodes that enable autonomous lane-following on the Freenove 4WD Smart Car:

1. **camera_node.py** - Captures images from Raspberry Pi Camera
2. **lane_follower_node.py** - Detects lanes using computer vision and calculates steering
3. **motor_control_node.py** - Controls physical motors based on velocity commands

### System Architecture

```
┌─────────────┐    /camera/image_raw    ┌──────────────┐
│ Camera Node │ ───────────────────────> │ Lane Follower│
└─────────────┘                          │     Node     │
                                         └──────────────┘
                                               │
                                               │ /cmd_vel (Twist)
                                               ▼
                                         ┌──────────────┐
                                         │ Motor Control│
                                         │     Node     │
                                         └──────────────┘
```

---

## 🔧 Prerequisites

- Raspberry Pi 4 (or compatible)
- Freenove 4WD Smart Car Kit assembled and tested
- ROS 2 Jazzy installed
- Python 3.10+
- Freenove motor control library installed

---

## 📦 Installation

### 1. Clone this repository

```bash
cd ~
git clone [YOUR_GIT_URL_HERE] freenove-ros2-nodes
```

### 2. Create ROS 2 workspace and package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create freenove_car \
  --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge
```

### 3. Copy node files to package

```bash
cd ~/ros2_ws/src/freenove_car/freenove_car
cp ~/freenove-ros2-nodes/*.py .
chmod +x camera_node.py lane_follower_node.py motor_control_node.py
```

### 4. Configure setup.py

Edit `~/ros2_ws/src/freenove_car/setup.py`:

```python
entry_points={
    'console_scripts': [
        'camera_node = freenove_car.camera_node:main',
        'lane_follower_node = freenove_car.lane_follower_node:main',
        'motor_control_node = freenove_car.motor_control_node:main',
    ],
},
```

### 5. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 🚀 Usage

### Running Individual Nodes

You'll need **3 separate terminal windows**, each SSH'd into your robot.

**Terminal 1 - Camera Node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car camera_node
```

**Terminal 2 - Lane Follower Node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run freenove_car lane_follower_node
```

**Terminal 3 - Motor Control Node:**
```bash
source ~/ros2_ws/install/setup.bash
sudo -E ros2 run freenove_car motor_control_node
```

> **Note:** The motor control node requires `sudo` for GPIO access. The `-E` flag preserves environment variables so ROS 2 can find topics.

### Emergency Stop

Press `Ctrl+C` in the motor control terminal (Terminal 3) to immediately stop the robot.

---

## 📊 ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/freenove/camera/image_raw` | `sensor_msgs/Image` | Raw camera images (30 Hz) |
| `/freenove/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

---

## 🔍 Debugging Tools

### Check running nodes
```bash
ros2 node list
```

### Check active topics
```bash
ros2 topic list
```

### Monitor camera images
```bash
ros2 topic hz /freenove/camera/image_raw
```

### View velocity commands
```bash
ros2 topic echo /freenove/cmd_vel
```

### Visualize system architecture
```bash
ros2 run rqt_graph rqt_graph
```

### Inspect node details
```bash
ros2 node info /lane_follower
```

---

## 🎛️ Configuration

### Lane Follower Parameters

Edit `lane_follower_node.py` to adjust:

```python
self.base_speed = 0.25              # Forward speed (m/s)
self.max_angular_speed = 0.5        # Max turning speed (rad/s)
self.steering_gain = 0.01           # Steering sensitivity
```

### Motor Control Parameters

Edit `motor_control_node.py` to adjust:

```python
self.base_speed = 100               # Base motor power (0-100)
self.turn_speed_factor = 0.5        # Turning aggressiveness
self.timeout_duration = 1.0         # Safety timeout (seconds)
```

---

## 📸 Computer Vision Pipeline

The lane follower uses these techniques from Session 3:

1. **Grayscale conversion** - Simplify image processing
2. **Gaussian blur** - Reduce noise
3. **Canny edge detection** - Find edges in the image
4. **Region of Interest (ROI)** - Focus on lower portion of image
5. **Hough line detection** - Find straight lines (lanes)
6. **Steering calculation** - Proportional control based on lane position

---

## 🐛 Troubleshooting

### Camera not working
- Check camera cable connection
- Enable camera: `sudo raspi-config` → Interface Options → Camera
- Verify with: `vcgencmd get_camera`
- Test with Freenove scripts first

### Motors not responding
- Check S1 and S2 power switches are ON
- Verify battery charge
- Make sure you're using `sudo -E` for motor control node
- Test motors with Freenove test script first

### Nodes can't find each other
- Make sure all terminals have sourced the workspace:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- Check topics exist: `ros2 topic list`
- Verify nodes are running: `ros2 node list`

### Lane detection not working
- Check camera is pointing at lane markings
- Ensure good lighting conditions
- High contrast between lanes and ground (white lines on dark surface)
- Adjust ROI vertices in `lane_follower_node.py` if needed

### Build errors
- Verify all dependencies in `package.xml`
- Check `setup.py` entry_points syntax
- Clean build: `rm -rf ~/ros2_ws/build ~/ros2_ws/install`
- Rebuild: `colcon build --symlink-install`

---

## 📚 Learning Resources

### ROS 2 Tutorials
- [ROS 2 Official Documentation](https://docs.ros.org/en/jazzy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

### Computer Vision
- [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [Canny Edge Detection](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html)
- [Hough Line Transform](https://docs.opencv.org/4.x/d9/db0/tutorial_hough_lines.html)

### Freenove Resources
- [Freenove GitHub](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi)
- [Freenove Documentation](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Tutorial.pdf)

---

## 🎓 Course Context

This code is part of **Engineering Teamwork III: AI and Autonomous Systems Lab** at Berlin University of Applied Sciences.

**Related Sessions:**
- Session 2: Introduction to ROS (TurtleSim)
- Session 3: Computer Vision Fundamentals (OpenCV)
- Session 4: Hardware Setup and Testing
- **Session 5: ROS 2 Integration** ← You are here
- Session 6+: Advanced autonomous behaviors

---

## 📝 Code Structure

```
freenove-ros2-nodes/
├── camera_node.py           # Camera capture and publishing
├── lane_follower_node.py    # Lane detection and control logic
├── motor_control_node.py    # Motor control interface
└── README.md                # This file
```

---

## 🤝 Contributing

Students are encouraged to:
- Improve lane detection algorithms
- Add obstacle detection
- Implement path planning
- Add visualization for debugging
- Optimize parameters for different track conditions

---

## ⚖️ License

Educational use only - Berlin University of Applied Sciences

---

## 📧 Support

For questions or issues:
1. Check the troubleshooting section above
2. Review Session 5 lab handout
3. Ask your instructor
4. Post in the course discussion forum

---

## 🎉 Acknowledgments

- Freenove for the excellent robotics platform
- ROS 2 community for comprehensive documentation
- OpenCV community for computer vision tools
- Course instructors and teaching assistants

---

**Good luck with your autonomous lane-following robot! 🤖**
