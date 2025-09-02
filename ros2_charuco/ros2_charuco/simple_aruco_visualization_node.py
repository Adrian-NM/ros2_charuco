#!/usr/bin/env python3
"""
Simple visualization node for ArUco markers.
Subscribes to raw image and detected markers, draws coordinate axes and IDs.
"""

import rclpy
import rclpy.node
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from ros2_charuco_interfaces.msg import CharucoMarkers


class SimpleArucoVisualizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("simple_aruco_visualization_node")
        
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.0225  # Default marker size
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.markers_sub = self.create_subscription(
            CharucoMarkers, "/charuco_markers", self.markers_callback, 10
        )
        
        # Publisher for visualization
        self.vis_pub = self.create_publisher(Image, "/aruco/visualization", 10)
        
        # Store latest data
        self.latest_image = None
        self.latest_markers = None
        
        self.get_logger().info("Simple Aruco Visualization Node initialized")
        
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
            return
        
        # Create a copy of the image for drawing
        vis_image = self.latest_image.copy()
        
        # Get marker data
        marker_ids = self.latest_markers.marker_ids
        poses = self.latest_markers.poses
        
        # Define object points for marker coordinate system
        # marker_size x marker_size square in marker coordinate system
        half_size = self.marker_size / 2
        obj_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        # Draw each marker
        for i, (marker_id, pose) in enumerate(zip(marker_ids, poses)):
            # Extract position and orientation from pose
            tvec = np.array([
                [pose.position.x],
                [pose.position.y],
                [pose.position.z]
            ])
            
            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation as R
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            rot_matrix = R.from_quat(quat).as_matrix()
            
            # Project marker corners to image plane
            img_points, _ = cv2.projectPoints(
                obj_points, rot_matrix, tvec, self.camera_matrix, self.dist_coeffs
            )
            
            # Draw marker outline
            img_points = img_points.astype(int)
            for j in range(4):
                cv2.line(vis_image, 
                        tuple(img_points[j][0]), 
                        tuple(img_points[(j+1)%4][0]), 
                        (0, 255, 0), 2)
            
            # Draw coordinate axes at marker center
            # Define axis endpoints in marker coordinate system
            axis_length = self.marker_size * 0.5
            axis_points = np.array([
                [0, 0, 0],
                [axis_length, 0, 0],  # X axis (red)
                [0, axis_length, 0],  # Y axis (green)
                [0, 0, axis_length]   # Z axis (blue)
            ], dtype=np.float32)
            
            # Project axes to image
            axis_img_points, _ = cv2.projectPoints(
                axis_points, rot_matrix, tvec, self.camera_matrix, self.dist_coeffs
            )
            axis_img_points = axis_img_points.astype(int)
            
            # Draw axes
            # X axis - Red
            cv2.line(vis_image,
                    tuple(axis_img_points[0][0]),
                    tuple(axis_img_points[1][0]),
                    (0, 0, 255), 3)
            
            # Y axis - Green  
            cv2.line(vis_image,
                    tuple(axis_img_points[0][0]),
                    tuple(axis_img_points[2][0]),
                    (0, 255, 0), 3)
            
            # Z axis - Blue
            cv2.line(vis_image,
                    tuple(axis_img_points[0][0]),
                    tuple(axis_img_points[3][0]),
                    (255, 0, 0), 3)
            
            # Draw ID text at marker center
            text_pos = tuple(axis_img_points[0][0])
            cv2.putText(vis_image, str(marker_id), text_pos,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Publish visualization image
        vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
        self.vis_pub.publish(vis_msg)


def main():
    rclpy.init()
    node = SimpleArucoVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()