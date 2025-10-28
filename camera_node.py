#!/usr/bin/env python3
"""
Camera Node for Freenove 4WD Smart Car
Captures images from Raspberry Pi Camera and publishes to ROS 2 topic

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    """
    ROS 2 Node that captures images from Pi Camera at 30 FPS
    and publishes them to /freenove/camera/image_raw topic
    """
    
    def __init__(self):
        super().__init__('camera')
        
        # Create publisher for camera images
        self.publisher_ = self.create_publisher(
            Image, 
            '/freenove/camera/image_raw', 
            10
        )
        
        # Initialize CV Bridge (converts between OpenCV and ROS images)
        self.bridge = CvBridge()
        
        # Open Pi Camera
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            raise RuntimeError('Camera not available')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.get_logger().info('Camera opened successfully')
        self.get_logger().info('Camera node started - publishing to /freenove/camera/image_raw')
        
        # Create timer to capture and publish images at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.frame_count = 0
    
    def timer_callback(self):
        """
        Timer callback that captures and publishes camera frames
        """
        ret, frame = self.cap.read()
        
        if ret:
            # Convert OpenCV image to ROS Image message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                
                # Publish the image
                self.publisher_.publish(ros_image)
                
                # Log every 30 frames (once per second at 30 Hz)
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Published {self.frame_count} frames')
            
            except Exception as e:
                self.get_logger().error(f'Failed to convert/publish image: {e}')
        else:
            self.get_logger().warn('Failed to capture frame from camera')
    
    def destroy_node(self):
        """
        Clean up camera resources when node is destroyed
        """
        self.get_logger().info('Shutting down camera node...')
        self.cap.release()
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the camera node
    """
    rclpy.init(args=args)
    
    try:
        camera_node = CameraNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
