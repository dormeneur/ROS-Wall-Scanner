#!/usr/bin/env python3
# Place this in: ~/catkin_ws/src/wall_scanner/scripts/capacitive_sensor_node.py

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Bool, Header
import random

class CapacitiveSensorNode:
    def __init__(self):
        rospy.init_node('capacitive_sensor_node', anonymous=True)
        
        # Publishers
        self.cap_pub = rospy.Publisher('/wall_scanner/capacitive_data', PointCloud, queue_size=10)
        
        # Subscribers  
        self.scan_sub = rospy.Subscriber('/wall_scanner/scan_command', Bool, self.scan_callback)
        
        # Parameters
        self.scan_rate = rospy.get_param('~scan_rate', 10.0)  # Hz
        self.baseline_capacitance = rospy.get_param('~baseline_capacitance', 12.5)  # pF
        self.noise_level = rospy.get_param('~noise_level', 0.3)  # pF
        self.num_sensors = rospy.get_param('~num_sensors', 3)  # Sensor array size
        
        # State
        self.is_scanning = False
        self.calibrated = False
        self.baseline_readings = []
        self.current_position = 0.0
        
        # Simulated object positions and properties
        self.objects = [
            {'type': 'pipe', 'position': 4.0, 'width': 0.05, 'cap_change': 8.5},
            {'type': 'stud', 'position': 6.0, 'width': 0.04, 'cap_change': 12.0},
            {'type': 'pipe', 'position': 9.0, 'width': 0.03, 'cap_change': 6.2}
        ]
        
        rospy.loginfo("Capacitive sensor node initialized")
        
    def scan_callback(self, msg):
        """Handle scan start/stop commands"""
        self.is_scanning = msg.data
        if self.is_scanning:
            rospy.loginfo("Capacitive Sensor: Starting scan")
            self.start_scanning()
        else:
            rospy.loginfo("Capacitive Sensor: Stopping scan")
            
    def calibrate(self):
        """Calibrate sensor array baseline"""
        rospy.loginfo("Calibrating capacitive sensor array...")
        self.baseline_readings = []
        
        for sensor_id in range(self.num_sensors):
            samples = []
            for i in range(50):
                reading = self.baseline_capacitance + random.gauss(0, self.noise_level)
                samples.append(reading)
                rospy.sleep(0.01)
            self.baseline_readings.append(np.mean(samples))
            
        self.calibrated = True
        rospy.loginfo(f"Capacitive sensors calibrated: {[f'{r:.2f}' for r in self.baseline_readings]} pF")
        
    def get_capacitive_readings(self, position):
        """Simulate capacitive readings from sensor array"""
        readings = []
        sensor_spacing = 0.02  # 2cm between sensors
        
        for sensor_id in range(self.num_sensors):
            sensor_pos = position + (sensor_id - self.num_sensors//2) * sensor_spacing
            base_reading = self.baseline_readings[sensor_id] + random.gauss(0, self.noise_level)
            
            # Check for nearby objects
            for obj in self.objects:
                distance = abs(sensor_pos - obj['position'])
                if distance < obj['width'] + 0.1:  # Detection range
                    proximity = max(0, 1 - distance / (obj['width'] + 0.1))
                    base_reading += obj['cap_change'] * proximity
                    
            readings.append(max(0.0, base_reading))
            
        return readings
        
    def start_scanning(self):
        """Start the scanning loop"""
        if not self.calibrated:
            self.calibrate()
            
        rate = rospy.Rate(self.scan_rate)
        self.current_position = 0.0
        
        while self.is_scanning and not rospy.is_shutdown():
            # Simulate scanner moving along wall
            self.current_position += 0.02  # 2cm per reading
            if self.current_position > 10.0:
                self.current_position = 0.0
                
            # Get sensor readings
            cap_readings = self.get_capacitive_readings(self.current_position)
            
            # Create PointCloud message (reusing for capacitive data)
            cloud_msg = PointCloud()
            cloud_msg.header = Header()
            cloud_msg.header.stamp = rospy.Time.now()
            cloud_msg.header.frame_id = "capacitive_sensor_array"
            
            # Add points representing sensor readings
            for i, reading in enumerate(cap_readings):
                point = Point32()
                point.x = self.current_position
                point.y = i * 0.02  # Sensor position in array
                point.z = reading   # Capacitance value
                cloud_msg.points.append(point)
                
            self.cap_pub.publish(cloud_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = CapacitiveSensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass