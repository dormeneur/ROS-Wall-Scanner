#!/usr/bin/env python3
# Place this in: ~/catkin_ws/src/wall_scanner/scripts/electromagnetic_sensor_node.py

import rospy
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Header
import random

class ElectromagneticSensorNode:
    def __init__(self):
        rospy.init_node('electromagnetic_sensor_node', anonymous=True)
        
        # Publishers
        self.em_pub = rospy.Publisher('/wall_scanner/electromagnetic_data', Range, queue_size=10)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/wall_scanner/scan_command', Bool, self.scan_callback)
        
        # Parameters
        self.scan_rate = rospy.get_param('~scan_rate', 10.0)  # Hz
        self.baseline_field = rospy.get_param('~baseline_field', 0.1)  # μT
        self.noise_level = rospy.get_param('~noise_level', 0.05)  # μT
        
        # State
        self.is_scanning = False
        self.calibrated = False
        self.baseline_reading = 0.0
        
        # Simulated wire positions (in meters along wall)
        self.wire_positions = [2.0, 8.0]  # Live wires at these positions
        self.current_position = 0.0
        
        rospy.loginfo("Electromagnetic sensor node initialized")
        
    def scan_callback(self, msg):
        """Handle scan start/stop commands"""
        self.is_scanning = msg.data
        if self.is_scanning:
            rospy.loginfo("EM Sensor: Starting scan")
            self.start_scanning()
        else:
            rospy.loginfo("EM Sensor: Stopping scan")
            
    def calibrate(self):
        """Calibrate sensor baseline"""
        rospy.loginfo("Calibrating electromagnetic sensor...")
        samples = []
        for i in range(50):
            samples.append(self.baseline_field + random.gauss(0, self.noise_level))
            rospy.sleep(0.02)
        
        self.baseline_reading = np.mean(samples)
        self.calibrated = True
        rospy.loginfo(f"EM Sensor calibrated. Baseline: {self.baseline_reading:.3f} μT")
        
    def get_em_reading(self, position):
        """Simulate electromagnetic field reading at given position"""
        base_reading = self.baseline_reading + random.gauss(0, self.noise_level)
        
        # Add signal from nearby wires
        for wire_pos in self.wire_positions:
            distance = abs(position - wire_pos)
            if distance < 0.5:  # Detection range: 50cm
                # Inverse square law approximation
                signal_strength = 15.0 / (1 + distance * 10)
                base_reading += signal_strength
                
        return max(0.0, base_reading)
        
    def start_scanning(self):
        """Start the scanning loop"""
        if not self.calibrated:
            self.calibrate()
            
        rate = rospy.Rate(self.scan_rate)
        self.current_position = 0.0
        
        while self.is_scanning and not rospy.is_shutdown():
            # Simulate scanner moving along wall (10m wall)
            self.current_position += 0.02  # 2cm per reading
            if self.current_position > 10.0:
                self.current_position = 0.0
                
            # Get sensor reading
            em_value = self.get_em_reading(self.current_position)
            
            # Create and publish Range message
            range_msg = Range()
            range_msg.header = Header()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "electromagnetic_sensor"
            range_msg.radiation_type = Range.ULTRASOUND  # Reusing for EM field
            range_msg.field_of_view = 0.1  # 10cm field of view
            range_msg.min_range = 0.0
            range_msg.max_range = 50.0  # Max field strength
            range_msg.range = em_value
            
            self.em_pub.publish(range_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ElectromagneticSensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass