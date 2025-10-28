#!/usr/bin/env python3
"""
Lane Follower Node for Freenove 4WD Smart Car
Processes camera images to detect lanes and publishes steering commands

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration

Computer Vision Pipeline:
1. Receive image from camera node
2. Convert to grayscale
3. Apply Gaussian blur
4. Canny edge detection
5. Define region of interest (ROI)
6. Hough line detection
7. Calculate steering angle
8. Publish Twist command
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneFollowerNode(Node):
    """
    ROS 2 Node that detects lanes using computer vision
    and publishes velocity commands to follow them
    """
    
    def __init__(self):
        super().__init__('lane_follower')
        
        # Create subscriber to camera images
        self.subscription = self.create_subscription(
            Image,
            '/freenove/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(
            Twist,
            '/freenove/cmd_vel',
            10
        )
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Lane detection parameters
        self.base_speed = 0.25  # Forward speed (m/s)
        self.max_angular_speed = 0.5  # Maximum turning speed (rad/s)
        
        # Control parameters
        self.steering_gain = 0.01  # How aggressively to steer
        
        self.get_logger().info('Lane follower node started')
        self.get_logger().info('Subscribing to: /freenove/camera/image_raw')
        self.get_logger().info('Publishing to: /freenove/cmd_vel')
    
    def region_of_interest(self, img, vertices):
        """
        Apply mask to keep only the region of interest (lower portion of image)
        """
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def detect_lane_lines(self, image):
        """
        Detect lane lines using Canny edge detection and Hough transform
        
        Returns:
            lines: Detected lines from Hough transform
            processed_image: Image showing detection pipeline (for debugging)
        """
        height, width = image.shape[:2]
        
        # 1. Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 2. Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 3. Canny edge detection (Session 3 technique!)
        edges = cv2.Canny(blur, 50, 150)
        
        # 4. Define region of interest (trapezoid in lower portion of image)
        roi_vertices = np.array([[
            (width * 0.1, height),           # Bottom left
            (width * 0.4, height * 0.6),     # Top left
            (width * 0.6, height * 0.6),     # Top right
            (width * 0.9, height)            # Bottom right
        ]], dtype=np.int32)
        
        roi_edges = self.region_of_interest(edges, roi_vertices)
        
        # 5. Hough line detection (Session 3 technique!)
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=50,
            maxLineGap=150
        )
        
        return lines, edges
    
    def calculate_steering(self, lines, image_width):
        """
        Calculate steering angle based on detected lane lines
        
        Returns:
            steering_error: How far off-center we are (negative = left, positive = right)
            lane_detected: Whether we successfully detected lanes
        """
        if lines is None or len(lines) == 0:
            return 0.0, False
        
        # Calculate the average x-position of all detected lines
        x_positions = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Use the bottom point (higher y value) of each line
            if y1 > y2:
                x_positions.append(x1)
            else:
                x_positions.append(x2)
        
        if len(x_positions) == 0:
            return 0.0, False
        
        # Calculate average lane position
        avg_x = np.mean(x_positions)
        
        # Calculate error from center
        image_center = image_width / 2
        steering_error = avg_x - image_center
        
        return steering_error, True
    
    def image_callback(self, msg):
        """
        Callback for receiving camera images
        Processes image and publishes steering commands
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            height, width = cv_image.shape[:2]
            
            # Detect lane lines
            lines, edges = self.detect_lane_lines(cv_image)
            
            # Calculate steering
            steering_error, lane_detected = self.calculate_steering(lines, width)
            
            # Create velocity command
            cmd = Twist()
            
            if lane_detected:
                # Move forward and steer based on error
                cmd.linear.x = self.base_speed
                
                # Apply proportional control
                angular_velocity = -steering_error * self.steering_gain
                
                # Clamp angular velocity to maximum
                angular_velocity = max(min(angular_velocity, self.max_angular_speed), 
                                      -self.max_angular_speed)
                
                cmd.angular.z = angular_velocity
                
                # Log status
                self.get_logger().info(
                    f'Lane detected, steering_error: {steering_error:.2f}, '
                    f'angular_vel: {angular_velocity:.3f}',
                    throttle_duration_sec=1.0  # Log at most once per second
                )
            else:
                # No lane detected - stop or search
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                
                self.get_logger().warn(
                    'No lane detected! Stopping.',
                    throttle_duration_sec=1.0
                )
            
            # Publish command
            self.publisher_.publish(cmd)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    """
    Main function to initialize and run the lane follower node
    """
    rclpy.init(args=args)
    
    try:
        lane_follower = LaneFollowerNode()
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
