#!/usr/bin/env python3
"""
Visualization node for ArUco markers.
Draws coordinate axes and IDs on detected markers.
"""

import rclpy
import rclpy.node
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from ros2_charuco_interfaces.msg import CharucoMarkers


class ArucoVisualizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_visualization_node")
        
        # CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters (will be updated from camera info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Parameters
        self.declare_parameter("marker_size", 0.0225)
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        
        self.declare_parameter("input_image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("markers_topic", "/charuco_markers")
        self.declare_parameter("output_image_topic", "/aruco/visualization")
        
        input_image_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        markers_topic = self.get_parameter("markers_topic").get_parameter_value().string_value
        output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, input_image_topic, self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )
        self.markers_sub = self.create_subscription(
            CharucoMarkers, markers_topic, self.markers_callback, 10
        )
        
        # Publishers
        self.vis_pub = self.create_publisher(Image, output_image_topic, 10)
        
        # Store latest data
        self.latest_image = None
        self.latest_markers = None
        
        self.get_logger().info("Aruco Visualization Node initialized")
        
    def camera_info_callback(self, msg):
        """Update camera intrinsic parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info received")
    
    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_visualization()
    
    def markers_callback(self, msg):
        """Store latest marker detections"""
        self.latest_markers = msg
        self.process_visualization()
    
    def process_visualization(self):
        """Process visualization when both image and markers are available"""
        if self.latest_image is None or self.latest_markers is None:
            return
            
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera info...")
            return
            
        # Create a copy of the image for drawing
        vis_image = self.latest_image.copy()
        
        # Get marker data
        marker_ids = self.latest_markers.marker_ids
        poses = self.latest_markers.poses
        
        # Draw each marker
        for i, (marker_id, pose) in enumerate(zip(marker_ids, poses)):
            # Convert pose to OpenCV format
            rvec, tvec = self.pose_to_rvec_tvec(pose)
            
            # Draw coordinate axes
            cv2.drawFrameAxes(
                vis_image, 
                self.camera_matrix, 
                self.dist_coeffs, 
                rvec, 
                tvec, 
                self.marker_size * 0.5  # Axis length
            )
            
            # Get marker corners for ID text position
            # Project the marker center to image coordinates
            marker_center_3d = np.array([[0.0], [0.0], [0.0]])  # Center of marker
            # Ensure rvec is the correct shape (3, 1)
            rvec_flat = rvec.reshape(3, 1)
            img_points, _ = cv2.projectPoints(
                marker_center_3d, rvec_flat, tvec, self.camera_matrix, self.dist_coeffs
            )
            
            # Draw ID text
            text_pos = (int(img_points[0][0][0]), int(img_points[0][0][1]))
            cv2.putText(
                vis_image,
                str(marker_id),
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,  # Font scale
                (0, 255, 0),  # Green color
                1  # Thickness
            )
        
        # Publish visualization image
        vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
        vis_msg.header = self.latest_image.header
        self.vis_pub.publish(vis_msg)
    
    def pose_to_rvec_tvec(self, pose):
        """Convert ROS Pose to OpenCV rvec and tvec"""
        # Extract position
        tvec = np.array([
            [pose.position.x],
            [pose.position.y],
            [pose.position.z]
        ])
        
        # Extract orientation (quaternion to rotation matrix)
        from scipy.spatial.transform import Rotation as R
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        rot_matrix = R.from_quat(quat).as_matrix()
        
        # Convert rotation matrix to rvec
        rvec, _ = cv2.Rodrigues(rot_matrix)
        
        return rvec, tvec


def main():
    rclpy.init()
    node = ArucoVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()