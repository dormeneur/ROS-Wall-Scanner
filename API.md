# Wall Scanner API Reference

This document provides detailed API reference for the ROS Wall Scanner system.

## Table of Contents
- [ROS Topics](#ros-topics)
- [Custom Messages](#custom-messages)
- [ROS Services](#ros-services)
- [ROS Parameters](#ros-parameters)
- [Node APIs](#node-apis)
- [Hardware Interface](#hardware-interface)

---

## ROS Topics

### Published Topics

#### `/wall_scanner/electromagnetic_data`
**Type:** `sensor_msgs/Range`  
**Publisher:** `electromagnetic_sensor_node`  
**Rate:** 10 Hz  
**Description:** Electromagnetic field strength measurements for live wire detection

**Message Fields:**
```yaml
header:
  stamp: time        # Timestamp of measurement
  frame_id: string   # "electromagnetic_sensor"
radiation_type: uint8  # ULTRASOUND (repurposed for EM)
field_of_view: float32 # Detection cone angle (0.1 rad)
min_range: float32     # 0.0 μT
max_range: float32     # 50.0 μT maximum detectable field
range: float32         # Current EM field strength in μT
```

**Usage Example:**
```bash
rostopic echo /wall_scanner/electromagnetic_data
```

---

#### `/wall_scanner/capacitive_data`
**Type:** `sensor_msgs/PointCloud`  
**Publisher:** `capacitive_sensor_node`  
**Rate:** 10 Hz  
**Description:** Capacitive sensor array readings for pipe and stud detection

**Message Fields:**
```yaml
header:
  stamp: time        # Timestamp of measurement
  frame_id: string   # "capacitive_sensor_array"
points: Point32[]    # Array of sensor readings
  - x: float32       # Position along wall (meters)
  - y: float32       # Sensor position in array (0.02m spacing)
  - z: float32       # Capacitance value (pF)
```

**Array Layout:**
- **Point[0]:** Left sensor (y = 0.0)
- **Point[1]:** Center sensor (y = 0.02)
- **Point[2]:** Right sensor (y = 0.04)

---

#### `/wall_scanner/detection_results`
**Type:** `wall_scanner/DetectionResult`  
**Publisher:** `detection_fusion_node`  
**Rate:** On detection event  
**Description:** Processed detection results with object classification

**Message Structure:** See [Custom Messages](#detectionresult) section

---

#### `/wall_scanner/visualization_markers`
**Type:** `visualization_msgs/MarkerArray`  
**Publisher:** `visualization_node`  
**Rate:** On detection event  
**Description:** 3D visualization markers for RViz display

---

### Subscribed Topics

#### `/wall_scanner/scan_command`
**Type:** `std_msgs/Bool`  
**Subscribers:** `electromagnetic_sensor_node`, `capacitive_sensor_node`  
**Description:** Start/stop scanning command

**Usage:**
```bash
# Start scanning
rostopic pub /wall_scanner/scan_command std_msgs/Bool "data: true"

# Stop scanning
rostopic pub /wall_scanner/scan_command std_msgs/Bool "data: false"
```

---

#### `/wall_scanner/calibrate`
**Type:** `std_msgs/Empty`  
**Subscribers:** `electromagnetic_sensor_node`, `capacitive_sensor_node`  
**Description:** Trigger sensor calibration

**Usage:**
```bash
rostopic pub /wall_scanner/calibrate std_msgs/Empty "{}"
```

---

## Custom Messages

### DetectionResult
**File:** `msg/DetectionResult.msg`  
**Description:** Complete detection result with object classification and confidence

```yaml
# Object type constants
int32 WIRE = 0    # Live electrical wire
int32 PIPE = 1    # Water pipe
int32 STUD = 2    # Wall stud

# Detection data
int32 object_type          # Object type (WIRE/PIPE/STUD)
float64 position_x         # Position along wall (meters)
float64 position_y         # Estimated depth behind wall (meters)
float64 confidence         # Detection confidence (0.0-1.0)
string object_name         # Human-readable description
geometry_msgs/Point location  # 3D coordinates
builtin_interfaces/Time timestamp  # Detection timestamp
```

**Confidence Levels:**
- **0.7-1.0:** High confidence (reliable detection)
- **0.5-0.7:** Medium confidence (possible detection)
- **0.0-0.5:** Low confidence (filtered out)

**Position Accuracy:**
- **X-axis (along wall):** ±2cm
- **Y-axis (depth):** ±1cm for pipes, 0cm for studs

---

## ROS Services

### Manual Control Services

#### `/wall_scanner/start_scan`
**Type:** `std_srvs/Empty`  
**Description:** Start wall scanning process

```bash
rosservice call /wall_scanner/start_scan
```

#### `/wall_scanner/stop_scan`
**Type:** `std_srvs/Empty`  
**Description:** Stop wall scanning process

```bash
rosservice call /wall_scanner/stop_scan
```

#### `/wall_scanner/calibrate_service`
**Type:** `std_srvs/Empty`  
**Description:** Calibrate all sensors

```bash
rosservice call /wall_scanner/calibrate_service
```

---

## ROS Parameters

### Global Parameters

#### Detection Thresholds
```yaml
/wall_scanner/em_wire_threshold: 5.0      # μT - EM field for wire detection
/wall_scanner/cap_object_threshold: 15.0  # pF - Capacitance change threshold
/wall_scanner/confidence_threshold: 0.7   # Minimum confidence for valid detection
/wall_scanner/scan_rate: 10.0             # Hz - Sensor sampling rate
```

#### System Configuration
```yaml
/wall_scanner/max_detection_range: 0.5    # meters - Maximum detection distance
/wall_scanner/temporal_filter_size: 5     # samples - Moving average filter size
/wall_scanner/calibration_samples: 50     # Number of samples for baseline
```

### Node-Specific Parameters

#### Electromagnetic Sensor Node
```yaml
~scan_rate: 10.0           # Hz - Sampling frequency
~baseline_field: 0.1       # μT - Expected baseline EM field
~noise_level: 0.05         # μT - Sensor noise standard deviation
~detection_range: 0.5      # meters - Maximum detection range
```

#### Capacitive Sensor Node
```yaml
~scan_rate: 10.0           # Hz - Sampling frequency
~baseline_capacitance: 12.5 # pF - Baseline capacitance
~noise_level: 0.3          # pF - Sensor noise standard deviation
~num_sensors: 3            # Number of sensors in array
~sensor_spacing: 0.02      # meters - Distance between sensors
```

#### Detection Fusion Node
```yaml
~em_wire_threshold: 5.0    # μT - Wire detection threshold
~cap_object_threshold: 15.0 # pF - Object detection threshold
~confidence_threshold: 0.7  # Minimum confidence
~temporal_window: 5        # samples - Temporal filtering window
```

---

## Node APIs

### electromagnetic_sensor_node

#### Constructor Parameters
- **scan_rate** (float): Sensor sampling rate in Hz
- **baseline_field** (float): Expected baseline EM field in μT
- **noise_level** (float): Sensor noise level in μT

#### Public Methods
```python
def calibrate(self) -> None
    """Calibrate sensor baseline with 50 samples"""

def get_em_reading(self, position: float) -> float
    """Get EM field reading at specified position"""
    
def start_scanning(self) -> None
    """Start continuous scanning process"""
```

#### Callback Functions
```python
def scan_callback(self, msg: std_msgs.Bool) -> None
    """Handle scan start/stop commands"""
```

---

### capacitive_sensor_node

#### Constructor Parameters
- **scan_rate** (float): Sensor sampling rate in Hz
- **baseline_capacitance** (float): Baseline capacitance in pF
- **num_sensors** (int): Number of sensors in array

#### Public Methods
```python
def calibrate(self) -> None
    """Calibrate all sensors in array"""

def get_capacitive_readings(self, position: float) -> List[float]
    """Get readings from all sensors in array"""
```

---

### detection_fusion_node

#### Constructor Parameters
- **em_wire_threshold** (float): EM threshold for wire detection
- **cap_object_threshold** (float): Capacitance threshold for objects
- **confidence_threshold** (float): Minimum confidence for detection

#### Public Methods
```python
def fusion_callback(self, em_msg: Range, cap_msg: PointCloud) -> None
    """Process synchronized sensor data"""

def create_detection_result(self, obj_type: int, pos_x: float, 
                          pos_y: float, confidence: float, 
                          name: str) -> DetectionResult
    """Create formatted detection result message"""
```

#### Detection Logic
```python
# Wire Detection Algorithm
if em_field > em_wire_threshold:
    confidence = min(1.0, em_field / 20.0)
    if confidence > confidence_threshold:
        # Create wire detection

# Pipe/Stud Detection Algorithm  
if max_capacitance > cap_object_threshold:
    confidence = min(1.0, (max_capacitance - 12.5) / 15.0)
    if confidence > confidence_threshold:
        if max_capacitance > 20.0:
            object_type = STUD
        else:
            object_type = PIPE
```

---

## Hardware Interface

### Serial Communication Protocol

#### Connection Parameters
```yaml
Port: /dev/ttyUSB0
Baud Rate: 115200
Data Bits: 8
Parity: None
Stop Bits: 1
Flow Control: None
```

#### Message Format
**From Arduino to ROS:**
```
EM:<field_strength>,CAP:<cap1>,<cap2>,<cap3>,POS:<position>\n
```

**Example:**
```
EM:5.2,CAP:12.1,15.3,11.8,POS:1.25\n
```

#### Command Format
**From ROS to Arduino:**
```
CMD:<command>\n
```

**Commands:**
- `START` - Begin scanning
- `STOP` - Stop scanning  
- `CALIBRATE` - Calibrate sensors
- `RESET` - Reset system

---

## Error Codes and Troubleshooting

### Common Error Codes

#### Node Connection Errors
- **Error 1001:** Cannot connect to serial port
- **Error 1002:** Sensor calibration failed
- **Error 1003:** Message synchronization timeout

#### Detection Errors
- **Error 2001:** Low confidence detection filtered
- **Error 2002:** Sensor reading out of range
- **Error 2003:** Hardware communication timeout

#### System Errors
- **Error 3001:** ROS master not found
- **Error 3002:** Topic publishing failed
- **Error 3003:** Parameter server unreachable

### Debugging Commands

```bash
# Check node status
rosnode list
rosnode info electromagnetic_sensor

# Monitor topics
rostopic list
rostopic hz /wall_scanner/electromagnetic_data
rostopic echo /wall_scanner/detection_results

# View computation graph
rosrun rqt_graph rqt_graph

# Debug with logs
rosservice call /rosout/get_loggers
```

---

## Performance Specifications

### Timing Requirements
- **Sensor Sampling:** 10 Hz ±1 Hz
- **Detection Latency:** <200ms from sensor to result
- **Message Synchronization:** <100ms tolerance
- **Calibration Time:** 5 seconds for full calibration

### Accuracy Specifications
- **Position Accuracy:** ±2cm along wall, ±1cm depth
- **Detection Range:** 50cm maximum from scanner
- **False Positive Rate:** <5% under normal conditions
- **Detection Confidence:** >70% for reliable detections

### Resource Usage
- **CPU Usage:** <15% on single core
- **Memory Usage:** <100MB RAM per node
- **Network Bandwidth:** <1KB/s per topic
- **Storage:** <1MB for log files per hour