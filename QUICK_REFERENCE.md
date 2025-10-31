# ROS 2 Quick Reference Card
## Session 5 - Essential Commands

Keep this open while working! 📌

---

## 🚀 One-Time Setup

### Install ROS 2 (Run once)
```bash
wget https://raw.githubusercontent.com/RuthraBellan/freenove-ros2-setup/main/install_ros2.sh
chmod +x install_ros2.sh
./install_ros2.sh
```

---

## 🔄 Every Terminal Session

### Source ROS 2
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**💡 Tip:** Add to `~/.bashrc` to make automatic

---

## 🏗️ Building Your Package

### Build workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### After editing Python files
```bash
# No rebuild needed with --symlink-install!
# Just restart the node
```

### After editing setup.py or package.xml
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

## 🏃 Running Nodes

### Camera Node
```bash
ros2 run freenove_car camera_node
```

### Motor Control Node (needs sudo)
```bash
sudo -E ros2 run freenove_car motor_control_node
```

### Lane Follower Node
```bash
ros2 run freenove_car lane_follower_node
```

**💡 Tip:** Open 3 terminals (or use `tmux`/`screen`)

---

## 🔍 Debugging Commands

### What's running?
```bash
ros2 node list        # List all active nodes
ros2 topic list       # List all active topics
```

### What's being published?
```bash
ros2 topic echo /freenove/camera/image_raw --no-arr
ros2 topic echo /freenove/cmd_vel
```

### Topic details
```bash
ros2 topic info /freenove/camera/image_raw
ros2 topic hz /freenove/camera/image_raw    # Check frequency
```

### Node details
```bash
ros2 node info /camera
ros2 node info /motor_control
```

---

## 🧪 Testing Commands

### Manually send velocity command
```bash
# Move forward
ros2 topic pub /freenove/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /freenove/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /freenove/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## 🛑 Stop Everything

### Kill all ROS nodes
```bash
pkill -9 -f ros2
```

### Stop single node
```bash
Ctrl+C
```

---

## 🔧 Common Fixes

### "ros2: command not found"
```bash
source /opt/ros/humble/setup.bash
```

### "Package 'freenove_car' not found"
```bash
source ~/ros2_ws/install/setup.bash
```

### "Permission denied" (motors)
```bash
# Use sudo -E
sudo -E ros2 run freenove_car motor_control_node
```

### Camera not working
```bash
# Check if camera is enabled
ls /dev/video0

# If not found, reboot or check connections
```

### Rebuild everything
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

---

## 📂 Important Directories

```
~/ros2_ws/                              # Your workspace
├── src/
│   └── freenove_car/                   # Your package
│       ├── freenove_car/               # Python modules
│       │   ├── camera_node.py
│       │   ├── motor_control_node.py
│       │   └── lane_follower_node.py
│       ├── package.xml                 # Dependencies
│       └── setup.py                    # Entry points
├── build/                              # Build files
├── install/                            # Install files
└── log/                                # Build logs

~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/
└── Code/
    └── Server/
        └── motor.py                    # Motor control library
```

---

## 📝 Workflow Summary

1. **Edit code** → Save file
2. **Rebuild** (if needed) → `colcon build --symlink-install`
3. **Source workspace** → `source ~/ros2_ws/install/setup.bash`
4. **Run nodes** → `ros2 run freenove_car <node_name>`
5. **Debug** → `ros2 topic list`, `ros2 node list`
6. **Test** → Use `ros2 topic pub` or check output
7. **Stop** → `Ctrl+C` or `pkill -9 -f ros2`

---

## 💾 Save This!

**Bookmark this page or keep it open in a tab while working!**

Print it out? Even better! 📄

---

## 🆘 Need Help?

1. Check error messages carefully
2. Use debugging commands above
3. Review Session 5 PDF
4. Ask your team
5. Ask instructor

**Remember:** Everyone gets stuck sometimes. That's normal! 💪
