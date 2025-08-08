#!/usr/bin/env python3
# Place this in: ~/catkin_ws/src/wall_scanner/scripts/visualization_node.py

import rospy
from wall_scanner.msg import DetectionResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class VisualizationNode:
    def __init__(self):
        rospy.init_node('visualization_node', anonymous=True)
        
        # Publishers
        self.marker_pub = rospy.Publisher('/wall_scanner/visualization_markers', MarkerArray, queue_size=10)
        
        # Subscribers
        self.detection_sub = rospy.Subscriber('/wall_scanner/detection_results', DetectionResult, self.detection_callback)
        
        # Marker management
        self.markers = {}
        self.marker_id = 0
        
        # Colors for different object types
        self.colors = {
            DetectionResult.WIRE: ColorRGBA(1.0, 0.0, 0.0, 0.8),  # Red for wires
            DetectionResult.PIPE: ColorRGBA(0.0, 0.0, 1.0, 0.8),  # Blue for pipes  
            DetectionResult.STUD: ColorRGBA(0.6, 0.3, 0.0, 0.8)   # Brown for studs
        }
        
        rospy.loginfo("Visualization node initialized")
        
    def detection_callback(self, detection_msg):
        """Handle new detection results and create visualization markers"""
        
        # Create marker for detection
        marker = Marker()
        marker.header = detection_msg.header
        marker.header.frame_id = "wall"
        marker.ns = "detections"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = detection_msg.position_x
        marker.pose.position.y = detection_msg.position_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale based on object type
        if detection_msg.object_type == DetectionResult.WIRE:
            marker.scale.x = 0.02  # 2cm diameter
            marker.scale.y = 0.02
            marker.scale.z = 3.0   # 3m height
        elif detection_msg.object_type == DetectionResult.PIPE:
            marker.scale.x = 0.05  # 5cm diameter
            marker.scale.y = 0.05  
            marker.scale.z = 2.5   # 2.5m height
        else:  # STUD
            marker.scale.x = 0.04  # 4cm width
            marker.scale.y = 0.10  # 10cm depth
            marker.scale.z = 3.5   # 3.5m height
            
        # Color based on object type
        marker.color = self.colors.get(detection_msg.object_type, ColorRGBA(0.5, 0.5, 0.5, 0.8))
        
        # Adjust opacity based on confidence
        marker.color.a = 0.3 + 0.7 * detection_msg.confidence
        
        # Lifetime
        marker.lifetime = rospy.Duration(5.0)  # 5 seconds
        
        # Store marker
        key = f"{detection_msg.object_type}_{detection_msg.position_x:.2f}"
        self.markers[key] = marker
        
        # Create text marker for label
        text_marker = Marker()
        text_marker.header = detection_msg.header
        text_marker.header.frame_id = "wall"
        text_marker.ns = "detection_labels"
        text_marker.id = self.marker_id
        self.marker_id += 1
        
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Position above the object
        text_marker.pose.position.x = detection_msg.position_x
        text_marker.pose.position.y = detection_msg.position_y - 0.1
        text_marker.pose.position.z = 2.0
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.2  # Text height
        
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White text
        text_marker.text = f"{detection_msg.object_name}\n{detection_msg.confidence:.0%}"
        
        text_marker.lifetime = rospy.Duration(5.0)
        
        # Store text marker
        text_key = f"text_{key}"
        self.markers[text_key] = text_marker
        
        # Publish all markers
        self.publish_markers()
        
    def publish_markers(self):
        """Publish all current markers"""
        marker_array = MarkerArray()
        marker_array.markers = list(self.markers.values())
        
        # Add wall visualization
        wall_marker = self.create_wall_marker()
        marker_array.markers.append(wall_marker)
        
        # Add scanner position marker
        scanner_marker = self.create_scanner_marker()
        marker_array.markers.append(scanner_marker)
        
        self.marker_pub.publish(marker_array)
        
    def create_wall_marker(self):
        """Create a marker representing the wall"""
        wall = Marker()
        wall.header.stamp = rospy.Time.now()
        wall.header.frame_id = "wall"
        wall.ns = "wall"
        wall.id = 9999
        wall.type = Marker.CUBE
        wall.action = Marker.ADD
        
        # Wall dimensions (10m x 3m x 0.1m)
        wall.pose.position.x = 5.0  # Center at 5m
        wall.pose.position.y = 0.0
        wall.pose.position.z = 1.5  # Center at 1.5m height
        wall.pose.orientation.w = 1.0
        
        wall.scale.x = 10.0  # Length
        wall.scale.y = 0.1   # Thickness  
        wall.scale.z = 3.0   # Height
        
        wall.color = ColorRGBA(0.8, 0.8, 0.8, 0.3)  # Light gray, semi-transparent
        
        return wall
        
    def create_scanner_marker(self):
        """Create a marker representing the scanner device"""
        scanner = Marker()
        scanner.header.stamp = rospy.Time.now()
        scanner.header.frame_id = "wall"
        scanner.ns = "scanner"
        scanner.id = 9998
        scanner.type = Marker.CUBE
        scanner.action = Marker.ADD
        
        # Scanner position (moves along wall)
        # For simplicity, we'll show it at a fixed position
        # In a real implementation, you'd track the actual scanner position
        scanner.pose.position.x = 1.0
        scanner.pose.position.y = -0.2
        scanner.pose.position.z = 1.0
        scanner.pose.orientation.w = 1.0
        
        scanner.scale.x = 0.1
        scanner.scale.y = 0.15
        scanner.scale.z = 0.2
        
        scanner.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        
        return scanner

if __name__ == '__main__':
    try:
        node = VisualizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass