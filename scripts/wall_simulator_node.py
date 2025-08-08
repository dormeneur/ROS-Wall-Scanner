#!/usr/bin/env python3
# Place this in: ~/catkin_ws/src/wall_scanner/scripts/wall_simulator_node.py

import rospy
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Pose
import time

class WallSimulatorNode:
    def __init__(self):
        rospy.init_node('wall_simulator_node', anonymous=True)
        
        # Publishers
        self.scan_command_pub = rospy.Publisher('/wall_scanner/scan_command', Bool, queue_size=1)
        self.calibrate_pub = rospy.Publisher('/wall_scanner/calibrate', Empty, queue_size=1)
        
        # Simulation parameters
        self.scan_duration = rospy.get_param('~scan_duration', 30.0)  # seconds
        self.auto_scan = rospy.get_param('~auto_scan', True)
        self.scan_interval = rospy.get_param('~scan_interval', 35.0)  # seconds between scans
        
        rospy.loginfo("Wall simulator node initialized")
        
        # Start simulation
        if self.auto_scan:
            self.run_simulation()
        else:
            self.manual_mode()
            
    def run_simulation(self):
        """Run automatic scanning simulation"""
        rospy.loginfo("Starting automatic scanning simulation")
        
        # Initial calibration
        rospy.sleep(2.0)  # Wait for other nodes to initialize
        self.calibrate_sensors()
        
        while not rospy.is_shutdown():
            # Start scan
            rospy.loginfo("=== STARTING WALL SCAN ===")
            self.start_scan()
            
            # Let it scan for specified duration
            rospy.sleep(self.scan_duration)
            
            # Stop scan
            rospy.loginfo("=== STOPPING WALL SCAN ===")
            self.stop_scan()
            
            # Wait before next scan
            rospy.loginfo(f"Waiting {self.scan_interval}s before next scan...")
            rospy.sleep(self.scan_interval)
            
    def manual_mode(self):
        """Wait for manual commands"""
        rospy.loginfo("Manual mode - Use rosservice calls to control:")
        rospy.loginfo("  rosservice call /wall_scanner/start_scan")
        rospy.loginfo("  rosservice call /wall_scanner/stop_scan")
        rospy.loginfo("  rosservice call /wall_scanner/calibrate")
        
        # Set up services for manual control
        rospy.Service('/wall_scanner/start_scan', Empty, self.start_scan_service)
        rospy.Service('/wall_scanner/stop_scan', Empty, self.stop_scan_service)
        rospy.Service('/wall_scanner/calibrate_service', Empty, self.calibrate_service)
        
        rospy.spin()
        
    def start_scan_service(self, req):
        """Service callback to start scanning"""
        self.start_scan()
        return []
        
    def stop_scan_service(self, req):
        """Service callback to stop scanning"""
        self.stop_scan()
        return []
        
    def calibrate_service(self, req):
        """Service callback to calibrate sensors"""
        self.calibrate_sensors()
        return []
        
    def start_scan(self):
        """Start the scanning process"""
        msg = Bool()
        msg.data = True
        self.scan_command_pub.publish(msg)
        rospy.loginfo("Scan command sent: START")
        
    def stop_scan(self):
        """Stop the scanning process"""
        msg = Bool()
        msg.data = False
        self.scan_command_pub.publish(msg)
        rospy.loginfo("Scan command sent: STOP")
        
    def calibrate_sensors(self):
        """Calibrate all sensors"""
        rospy.loginfo("Calibrating sensors...")
        msg = Empty()
        self.calibrate_pub.publish(msg)
        rospy.loginfo("Calibration command sent")

if __name__ == '__main__':
    try:
        node = WallSimulatorNode()
    except rospy.ROSInterruptException:
        pass