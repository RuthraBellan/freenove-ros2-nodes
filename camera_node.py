#!/usr/bin/env python3
"""
Camera Node for Freenove 4WD Smart Car - Picamera2 Version
FULLY COMPATIBLE with Freenove hardware

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration

This version uses Picamera2 (the official Raspberry Pi camera library)
instead of OpenCV VideoCapture, matching Freenove's implementation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Import Picamera2 library
try:
    from picamera2 import Picamera2
    from libcamera import Transform
except ImportError as e:
    print(f"Error: Picamera2 not installed: {e}")
    print("Install with: sudo apt install -y python3-picamera2")
    import sys
    sys.exit(1)


class CameraNode(Node):
    """
    ROS 2 Node that captures images using Picamera2
    and publishes them to /freenove/camera/image_raw topic
    
    This implementation is fully compatible with Freenove hardware
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
        
        # Initialize Picamera2 (matching Freenove's approach)
        try:
            self.get_logger().info('Initializing Picamera2...')
            self.camera = Picamera2()
            
            # Configure camera with same settings as Freenove
            # RGB888 format for easy OpenCV conversion
            config = self.camera.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                transform=Transform(hflip=0, vflip=0)
            )
            self.camera.configure(config)
            
            # Start camera
            self.camera.start()
            
            self.get_logger().info('✓ Picamera2 initialized successfully')
            self.get_logger().info('  Resolution: 640x480')
            self.get_logger().info('  Format: RGB888')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Picamera2: {e}')
            self.get_logger().error('Make sure:')
            self.get_logger().error('  1. Camera is connected properly')
            self.get_logger().error('  2. Camera is enabled in raspi-config')
            self.get_logger().error('  3. Picamera2 is installed: sudo apt install python3-picamera2')
            raise
        
        # Create timer to capture and publish images at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        self.frame_count = 0
        self.get_logger().info('Camera node started - publishing to /freenove/camera/image_raw at 30 Hz')
    
    def timer_callback(self):
        """
        Timer callback that captures and publishes camera frames
        Called every 1/30 second (30 Hz)
        """
        try:
            # Capture frame as numpy array (RGB format)
            # This is the Picamera2 way - much simpler than OpenCV!
            frame_rgb = self.camera.capture_array()
            
            # Convert RGB to BGR for OpenCV/ROS compatibility
            # (OpenCV and cv_bridge expect BGR format)
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # Convert OpenCV image (numpy array) to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            
            # Publish the image
            self.publisher_.publish(ros_image)
            
            # Log progress (once per second = every 30 frames at 30 Hz)
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published {self.frame_count} frames',
                    throttle_duration_sec=1.0
                )
        
        except Exception as e:
            self.get_logger().error(f'Error capturing/publishing frame: {e}')
    
    def destroy_node(self):
        """
        Clean up camera resources when node is destroyed
        """
        self.get_logger().info('Shutting down camera node...')
        try:
            # Stop and close camera properly
            self.camera.stop()
            self.camera.close()
            self.get_logger().info('Camera closed successfully')
        except Exception as e:
            self.get_logger().warn(f'Error closing camera: {e}')
        
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
        print('\nShutting down camera node...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
