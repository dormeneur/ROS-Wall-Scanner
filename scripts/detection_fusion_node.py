#!/usr/bin/env python3
# Place this in: ~/catkin_ws/src/wall_scanner/scripts/detection_fusion_node.py

import rospy
import numpy as np
from sensor_msgs.msg import Range, PointCloud
from wall_scanner.msg import DetectionResult
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Header
import message_filters

class DetectionFusionNode:
    def __init__(self):
        rospy.init_node('detection_fusion_node', anonymous=True)
        
        # Publishers
        self.detection_pub = rospy.Publisher('/wall_scanner/detection_results', DetectionResult, queue_size=10)
        
        # Subscribers with message synchronization
        self.em_sub = message_filters.Subscriber('/wall_scanner/electromagnetic_data', Range)
        self.cap_sub = message_filters.Subscriber('/wall_scanner/capacitive_data', PointCloud)
        
        # Synchronize messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.em_sub, self.cap_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.fusion_callback)
        
        # Detection parameters
        self.em_wire_threshold = rospy.get_param('~em_wire_threshold', 5.0)  # μT
        self.cap_object_threshold = rospy.get_param('~cap_object_threshold', 15.0)  # pF
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.7)
        
        # Detection history for filtering
        self.detection_history = []
        self.max_history = 5
        
        rospy.loginfo("Detection fusion node initialized")
        
    def fusion_callback(self, em_msg, cap_msg):
        """Process synchronized sensor data and perform detection fusion"""
        
        # Extract position from first capacitive sensor point
        if not cap_msg.points:
            return
            
        position_x = cap_msg.points[0].x
        
        # Get EM field strength
        em_field = em_msg.range
        
        # Get capacitive readings (average across array)
        cap_readings = [point.z for point in cap_msg.points]
        avg_capacitance = np.mean(cap_readings)
        max_capacitance = np.max(cap_readings)
        
        # Detection logic
        detections = []
        
        # Wire detection (high EM field)
        if em_field > self.em_wire_threshold:
            confidence = min(1.0, em_field / 20.0)  # Scale confidence
            if confidence > self.confidence_threshold:
                detection = self.create_detection_result(
                    DetectionResult.WIRE, 
                    position_x, 
                    0.02,  # Estimated depth
                    confidence,
                    "Live electrical wire"
                )
                detections.append(detection)
                
        # Pipe/Stud detection (high capacitance change)
        if max_capacitance > self.cap_object_threshold:
            confidence = min(1.0, (max_capacitance - 12.5) / 15.0)  # Scale confidence
            if confidence > self.confidence_threshold:
                # Determine object type based on capacitance characteristics
                if max_capacitance > 20.0:
                    obj_type = DetectionResult.STUD
                    obj_name = "Wall stud"
                    depth = 0.0  # Studs are part of the wall
                else:
                    obj_type = DetectionResult.PIPE
                    obj_name = "Water pipe"
                    depth = 0.03  # Pipes are typically behind drywall
                    
                detection = self.create_detection_result(
                    obj_type,
                    position_x,
                    depth,
                    confidence, 
                    obj_name
                )
                detections.append(detection)
                
        # Apply temporal filtering to reduce false positives
        filtered_detections = self.filter_detections(detections, position_x)
        
        # Publish detections
        for detection in filtered_detections:
            self.detection_pub.publish(detection)
            rospy.loginfo(f"DETECTION: {detection.object_name} at {detection.position_x:.2f}m "
                         f"(confidence: {detection.confidence:.2f})")
    
    def create_detection_result(self, obj_type, pos_x, pos_y, confidence, name):
        """Create a DetectionResult message"""
        detection = DetectionResult()
        detection.header = Header()
        detection.header.stamp = rospy.Time.now()
        detection.header.frame_id = "wall_scanner"
        
        detection.object_type = obj_type
        detection.position_x = pos_x
        detection.position_y = pos_y
        detection.confidence = confidence
        detection.object_name = name
        
        detection.location = Point()
        detection.location.x = pos_x
        detection.location.y = pos_y
        detection.location.z = 0.0
        
        return detection
        
    def filter_detections(self, detections, current_pos):
        """Apply temporal filtering to reduce noise"""
        # Add current detections to history
        self.detection_history.append({
            'position': current_pos,
            'detections': detections,
            'timestamp': rospy.Time.now()
        })
        
        # Keep only recent history
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
            
        # For now, just return current detections
        # In a real system, you'd implement more sophisticated filtering
        return detections

if __name__ == '__main__':
    try:
        node = DetectionFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass