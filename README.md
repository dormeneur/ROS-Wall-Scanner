# Wall Scanner Bot
ROS project to detect water pipelines and electric wires in any wall.

<img width="300" height="250" alt="Screenshot 2025-08-07 225639" src="https://github.com/user-attachments/assets/51e349b5-c19a-4014-afa9-a270d6e03503" />
<img width="300" height="250" alt="Screenshot 2025-08-07 225535" src="https://github.com/user-attachments/assets/0dc28bf3-18bc-46ea-9113-47f151dc276a" />

# Wall Scanner ROS Setup Instructions

## 1. Prerequisites
- ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
- Python 3
- catkin workspace set up

## 2. Package Setup

### Create the package structure:
```bash
cd ~/catkin_ws/src
catkin_create_pkg wall_scanner rospy std_msgs sensor_msgs geometry_msgs visualization_msgs
cd wall_scanner
mkdir launch config scripts msg
```

### File Placement:
Place files in the following locations:

```
~/catkin_ws/src/wall_scanner/
├── msg/
│   └── DetectionResult.msg
├── scripts/
│   ├── electromagnetic_sensor_node.py
│   ├── capacitive_sensor_node.py  
│   ├── detection_fusion_node.py
│   ├── visualization_node.py
│   └── wall_simulator_node.py
├── launch/
│   └── wall_scanner.launch
├── config/
│   └── wall_scanner.rviz
├── package.xml
└── CMakeLists.txt
```

### Make scripts executable:
```bash
cd ~/catkin_ws/src/wall_scanner/scripts
chmod +x *.py
```

## 3. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. Running the Simulation

### Option 1: Full Launch (Recommended)
```bash
roslaunch wall_scanner wall_scanner.launch
```
This starts all nodes including RViz visualization.

### Option 2: Individual Nodes
Terminal 1 - Core:
```bash
roscore
```

Terminal 2 - Electromagnetic Sensor:
```bash
rosrun wall_scanner electromagnetic_sensor_node.py
```

Terminal 3 - Capacitive Sensor:
```bash
rosrun wall_scanner capacitive_sensor_node.py
```

Terminal 4 - Detection Fusion:
```bash
rosrun wall_scanner detection_fusion_node.py
```

Terminal 5 - Visualization:
```bash
rosrun wall_scanner visualization_node.py
```

Terminal 6 - Simulator:
```bash
rosrun wall_scanner wall_simulator_node.py
```

Terminal 7 - RViz:
```bash
rosrun rviz rviz -d ~/catkin_ws/src/wall_scanner/config/wall_scanner.rviz
```

## 5. Monitoring the System

### View ROS Topics:
```bash
rostopic list
```
You should see:
- `/wall_scanner/electromagnetic_data`
- `/wall_scanner/capacitive_data`
- `/wall_scanner/detection_results`
- `/wall_scanner/scan_command`
- `/wall_scanner/visualization_markers`

### Monitor Detections:
```bash
rostopic echo /wall_scanner/detection_results
```

### View Sensor Data:
```bash
rostopic echo /wall_scanner/electromagnetic_data
rostopic echo /wall_scanner/capacitive_data
```

### Manual Control:
```bash
# Start scanning
rostopic pub /wall_scanner/scan_command std_msgs/Bool "data: true"

# Stop scanning  
rostopic pub /wall_scanner/scan_command std_msgs/Bool "data: false"

# Calibrate
rostopic pub /wall_scanner/calibrate std_msgs/Empty "{}"
```

## 6. Expected Behavior

1. **Initialization**: All nodes start and calibrate sensors
2. **Automatic Scanning**: Scanner moves along a 10m wall every 35 seconds
3. **Detections**: System detects:
   - Live wires at 2.0m and 8.0m (red cylinders in RViz)
   - Pipes at 4.0m and 9.0m (blue cylinders)
   - Stud at 6.0m (brown cylinder)
4. **Visualization**: RViz shows wall, scanner position, and detected objects
5. **Console Output**: Detection messages with confidence levels

## 7. Troubleshooting

### Python Import Errors:
```bash
export PYTHONPATH="${PYTHONPATH}:~/catkin_ws/devel/lib/python3/dist-packages"
```

### Message Generation Issues:
Ensure `message_generation` is in `build_depend` and `message_runtime` in `exec_depend` in package.xml.

### RViz Not Showing Markers:
- Check that `/wall_scanner/visualization_markers` topic has data
- Verify frame_id is set to "wall" in RViz Fixed Frame
- Ensure MarkerArray display is enabled

### No Detections:
- Check sensor thresholds in launch file
- Monitor raw sensor data topics
- Verify detection_fusion_node is receiving synchronized messages

## 8. Customization

### Modify Detection Parameters:
Edit `launch/wall_scanner.launch`:
```xml
<rosparam param="/wall_scanner/em_wire_threshold">5.0</rosparam>
<rosparam param="/wall_scanner/cap_object_threshold">15.0</rosparam>
<rosparam param="/wall_scanner/confidence_threshold">0.7</rosparam>
```

### Add New Objects:
Edit `electromagnetic_sensor_node.py` and `capacitive_sensor_node.py`:
```python
# Add new wire positions
self.wire_positions = [1.5, 2.0, 5.5, 8.0, 9.2]

# Add new object types in capacitive sensor
self.objects.append({
    'type': 'pipe', 
    'position': 3.5, 
    'width': 0.08, 
    'cap_change': 10.0
})
```

### Change Scan Behavior:
Modify `wall_simulator_node.py` parameters:
```python
self.scan_duration = 45.0    # Longer scans
self.scan_interval = 60.0    # More time between scans
```

## 9. Real Hardware Integration

To connect real hardware:
1. Replace simulation in sensor nodes with actual sensor readings
2. Add serial communication for Arduino/ESP32
3. Modify position tracking for physical scanner movement
4. Calibrate thresholds based on real sensor characteristics

## 10. ROS Network

For distributed systems:
```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.101
```

This complete ROS implementation provides a robust foundation for your wall scanner project with proper sensor fusion, visualization, and extensibility for real hardware integration.
