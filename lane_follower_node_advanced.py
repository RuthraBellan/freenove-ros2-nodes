#!/usr/bin/env python3
"""
ADVANCED Lane Follower Node for Freenove 4WD Smart Car
Enhanced version with proper lane detection, PID control, and search behavior

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration - Advanced Version

KEY IMPROVEMENTS OVER BASIC VERSION:
1. ✅ Separates left and right lane lines (not just averaging)
2. ✅ PID control (not just proportional)
3. ✅ Searches when lane lost (doesn't stop completely)
4. ✅ Adaptive speed (slows on curves)
5. ✅ Better ROI (looks further ahead)
6. ✅ Filters noise (horizontal lines, outliers)

Computer Vision Pipeline:
1. Receive image from camera node
2. Convert to grayscale
3. Apply Gaussian blur
4. Canny edge detection
5. Define region of interest (ROI)
6. Hough line detection
7. CLASSIFY lines into left/right lanes
8. Calculate steering angle with PID
9. Publish Twist command
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class LaneFollowerAdvancedNode(Node):
    """
    Advanced ROS 2 Node that detects lanes using computer vision
    with left/right lane separation and PID control
    """
    
    def __init__(self):
        super().__init__('lane_follower_advanced')
        
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
        
        # ============ SPEED PARAMETERS ============
        self.base_speed = 0.20          # Base forward speed (m/s)
        self.min_speed = 0.10           # Minimum speed on sharp turns
        self.max_angular_speed = 0.6    # Maximum turning speed (rad/s)
        
        # ============ PID PARAMETERS ============
        self.Kp = 0.012   # Proportional gain (increased from basic 0.01)
        self.Ki = 0.001   # Integral gain (NEW - handles steady-state error)
        self.Kd = 0.008   # Derivative gain (NEW - reduces oscillation)
        
        # PID state variables
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        
        # ============ LANE DETECTION PARAMETERS ============
        self.assumed_lane_width = 200   # Assumed lane width in pixels
        self.min_line_slope = 0.5       # Minimum slope to be considered a lane line
        
        # ============ SEARCH BEHAVIOR PARAMETERS ============
        self.frames_without_lane = 0
        self.max_frames_without_lane = 5    # Frames before entering search mode
        self.last_steering_error = 0.0
        self.search_direction = 1.0         # 1 = right, -1 = left
        
        # ============ SMOOTHING ============
        # Use deque for moving average of steering
        self.steering_history = deque(maxlen=3)
        
        self.get_logger().info('='*60)
        self.get_logger().info('ADVANCED Lane Follower Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Improvements:')
        self.get_logger().info('  ✅ Left/Right lane separation')
        self.get_logger().info('  ✅ PID control (Kp={}, Ki={}, Kd={})'.format(
            self.Kp, self.Ki, self.Kd))
        self.get_logger().info('  ✅ Search behavior when lane lost')
        self.get_logger().info('  ✅ Adaptive speed on curves')
        self.get_logger().info('  ✅ Better noise filtering')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribing to: /freenove/camera/image_raw')
        self.get_logger().info('Publishing to: /freenove/cmd_vel')
        self.get_logger().info('='*60)
    
    def region_of_interest(self, img, vertices):
        """
        Apply mask to keep only the region of interest
        """
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image
    
    def detect_lane_lines(self, image):
        """
        Detect lane lines using Canny edge detection and Hough transform
        IMPROVED: Larger ROI to see further ahead
        
        Returns:
            lines: Detected lines from Hough transform
            edges: Edge image (for debugging)
        """
        height, width = image.shape[:2]
        
        # 1. Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 2. Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 3. Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        
        # 4. IMPROVED ROI - Look further ahead (50% vs 60% in basic)
        roi_vertices = np.array([[
            (width * 0.0, height),           # Full width at bottom
            (width * 0.35, height * 0.5),    # Look at top 50% of image
            (width * 0.65, height * 0.5),    
            (width * 1.0, height)            
        ]], dtype=np.int32)
        
        roi_edges = self.region_of_interest(edges, roi_vertices)
        
        # 5. Hough line detection
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi/180,
            threshold=40,           # Slightly lower threshold
            minLineLength=40,       # Shorter minimum
            maxLineGap=150
        )
        
        return lines, edges
    
    def classify_lines(self, lines, image_width):
        """
        NEW FEATURE: Separate lines into left and right lanes
        
        This is the KEY improvement over basic version!
        
        Returns:
            left_lines: Lines belonging to left lane
            right_lines: Lines belonging to right lane
        """
        if lines is None or len(lines) == 0:
            return [], []
        
        image_center = image_width / 2
        left_lines = []
        right_lines = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Avoid division by zero
            if x2 - x1 == 0:
                continue
            
            # Calculate slope
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter out near-horizontal lines (noise, shadows)
            if abs(slope) < self.min_line_slope:
                continue
            
            # Calculate center of line
            center_x = (x1 + x2) / 2
            
            # Classify based on position AND slope
            # Left lane: negative slope (top-left to bottom-right), left of center
            # Right lane: positive slope (top-right to bottom-left), right of center
            
            if slope < 0 and center_x < image_center * 1.2:  # Allow some overlap
                left_lines.append((line, slope))
            elif slope > 0 and center_x > image_center * 0.8:
                right_lines.append((line, slope))
        
        return left_lines, right_lines
    
    def get_lane_x_position(self, lane_lines, at_y):
        """
        Calculate average x-position of a lane at given y-coordinate
        Uses weighted average based on line length
        
        Args:
            lane_lines: List of (line, slope) tuples
            at_y: Y-coordinate to evaluate (typically near bottom of image)
        """
        if len(lane_lines) == 0:
            return None
        
        weighted_x = 0
        total_weight = 0
        
        for line, slope in lane_lines:
            x1, y1, x2, y2 = line[0]
            
            # Calculate line length (weight)
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            
            # Extrapolate to at_y
            if abs(y2 - y1) > 1:
                # Use linear interpolation/extrapolation
                t = (at_y - y1) / (y2 - y1)
                x_at_y = x1 + t * (x2 - x1)
            else:
                x_at_y = (x1 + x2) / 2
            
            weighted_x += x_at_y * length
            total_weight += length
        
        if total_weight > 0:
            return weighted_x / total_weight
        return None
    
    def calculate_steering_advanced(self, lines, image_width, image_height):
        """
        IMPROVED: Calculate steering based on separated left and right lanes
        
        Returns:
            steering_error: How far off-center (pixels)
            lane_detected: Whether we found lanes
            confidence: How confident we are (0-1)
        """
        if lines is None or len(lines) == 0:
            return 0.0, False, 0.0
        
        # Classify lines into left and right
        left_lines, right_lines = self.classify_lines(lines, image_width)
        
        # Calculate where to evaluate (near bottom of image)
        eval_y = image_height * 0.8
        image_center = image_width / 2
        
        # Get x-positions of left and right lanes
        left_x = self.get_lane_x_position(left_lines, eval_y)
        right_x = self.get_lane_x_position(right_lines, eval_y)
        
        # CASE 1: Both lanes detected - BEST CASE
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
            steering_error = lane_center - image_center
            confidence = 1.0
            
            self.get_logger().debug(
                f'Both lanes: left={left_x:.0f}, right={right_x:.0f}, '
                f'center={lane_center:.0f}',
                throttle_duration_sec=1.0
            )
            return steering_error, True, confidence
        
        # CASE 2: Only left lane detected
        elif left_x is not None:
            # Assume right lane is offset by lane width
            estimated_center = left_x + (self.assumed_lane_width / 2)
            steering_error = estimated_center - image_center
            confidence = 0.7  # Less confident
            
            self.get_logger().debug(
                f'Left lane only: left={left_x:.0f}, estimated_center={estimated_center:.0f}',
                throttle_duration_sec=1.0
            )
            return steering_error, True, confidence
        
        # CASE 3: Only right lane detected
        elif right_x is not None:
            # Assume left lane is offset by lane width
            estimated_center = right_x - (self.assumed_lane_width / 2)
            steering_error = estimated_center - image_center
            confidence = 0.7  # Less confident
            
            self.get_logger().debug(
                f'Right lane only: right={right_x:.0f}, estimated_center={estimated_center:.0f}',
                throttle_duration_sec=1.0
            )
            return steering_error, True, confidence
        
        # CASE 4: No valid lanes
        else:
            return 0.0, False, 0.0
    
    def calculate_pid_control(self, steering_error):
        """
        NEW FEATURE: PID control instead of just proportional
        
        This reduces oscillation and handles steady-state error better
        
        Returns:
            angular_velocity: Calculated steering command
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0 or dt > 1.0:  # Sanity check
            dt = 0.033  # Assume 30 Hz
        
        # Proportional term
        P = self.Kp * steering_error
        
        # Integral term (with anti-windup)
        self.integral_error += steering_error * dt
        # Anti-windup: clamp integral
        max_integral = 100.0
        self.integral_error = max(min(self.integral_error, max_integral), -max_integral)
        I = self.Ki * self.integral_error
        
        # Derivative term
        if dt > 0:
            derivative = (steering_error - self.last_error) / dt
        else:
            derivative = 0.0
        D = self.Kd * derivative
        
        # Update state
        self.last_error = steering_error
        self.last_time = current_time
        
        # Combined output (note the negative sign for proper steering direction)
        angular_velocity = -(P + I + D)
        
        # Clamp to maximum
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), 
                              -self.max_angular_speed)
        
        return angular_velocity
    
    def calculate_adaptive_speed(self, steering_error, confidence):
        """
        NEW FEATURE: Slow down on sharp turns
        
        Returns:
            linear_speed: Adjusted forward speed
        """
        # Base speed
        speed = self.base_speed
        
        # Reduce speed based on steering error magnitude
        error_normalized = min(abs(steering_error) / 150.0, 1.0)  # Normalize to 0-1
        speed_reduction = 0.5 * error_normalized  # Up to 50% reduction
        
        speed = self.base_speed * (1.0 - speed_reduction)
        
        # Also reduce speed if low confidence
        speed = speed * (0.5 + 0.5 * confidence)
        
        # Enforce minimum speed
        speed = max(speed, self.min_speed)
        
        return speed
    
    def smooth_steering(self, angular_velocity):
        """
        Apply moving average to reduce jitter
        """
        self.steering_history.append(angular_velocity)
        return np.mean(self.steering_history)
    
    def image_callback(self, msg):
        """
        Main callback for receiving camera images
        Processes image and publishes steering commands
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            height, width = cv_image.shape[:2]
            
            # Detect lane lines
            lines, edges = self.detect_lane_lines(cv_image)
            
            # Calculate steering with advanced algorithm
            steering_error, lane_detected, confidence = self.calculate_steering_advanced(
                lines, width, height
            )
            
            # Create velocity command
            cmd = Twist()
            
            if lane_detected:
                # Reset lost counter
                self.frames_without_lane = 0
                self.last_steering_error = steering_error
                
                # Calculate PID control
                angular_velocity = self.calculate_pid_control(steering_error)
                
                # Smooth the steering
                angular_velocity = self.smooth_steering(angular_velocity)
                
                # Adaptive speed
                linear_speed = self.calculate_adaptive_speed(steering_error, confidence)
                
                cmd.linear.x = linear_speed
                cmd.angular.z = angular_velocity
                
                # Reset integral on lane detection to prevent windup
                if self.frames_without_lane > 0:
                    self.integral_error = 0.0
                
                # Log status
                self.get_logger().info(
                    f'Lane detected (conf={confidence:.2f}), '
                    f'error={steering_error:.1f}px, '
                    f'ang_vel={angular_velocity:.3f}, '
                    f'speed={linear_speed:.2f}',
                    throttle_duration_sec=1.0
                )
            
            else:
                # Increment lost counter
                self.frames_without_lane += 1
                
                if self.frames_without_lane <= self.max_frames_without_lane:
                    # Brief loss - keep going with last known direction
                    angular_velocity = self.calculate_pid_control(self.last_steering_error)
                    angular_velocity = self.smooth_steering(angular_velocity)
                    
                    cmd.linear.x = self.base_speed * 0.5  # Half speed
                    cmd.angular.z = angular_velocity
                    
                    self.get_logger().warn(
                        f'Lane briefly lost ({self.frames_without_lane}/{self.max_frames_without_lane}) '
                        f'- continuing last direction',
                        throttle_duration_sec=0.5
                    )
                
                else:
                    # Extended loss - enter search mode
                    cmd.linear.x = 0.0  # Stop moving forward
                    cmd.angular.z = 0.3 * self.search_direction  # Rotate slowly
                    
                    # Reset PID state
                    self.integral_error = 0.0
                    self.last_error = 0.0
                    
                    self.get_logger().warn(
                        f'Lane lost for {self.frames_without_lane} frames - SEARCHING',
                        throttle_duration_sec=1.0
                    )
                    
                    # Reverse search direction occasionally
                    if self.frames_without_lane > 30:  # 1 second
                        self.search_direction *= -1
                        self.frames_without_lane = self.max_frames_without_lane + 1
            
            # Publish command
            self.publisher_.publish(cmd)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    """
    Main function to initialize and run the advanced lane follower node
    """
    rclpy.init(args=args)
    
    try:
        lane_follower = LaneFollowerAdvancedNode()
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        print('\nShutting down advanced lane follower...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
