#!/usr/bin/env python3
"""
Motor Control Node for Freenove 4WD Smart Car
Subscribes to velocity commands and controls physical motors

Course: Engineering Teamwork III - AI and Autonomous Systems Lab
Session 5: ROS 2 Integration

This node bridges ROS 2 to the Freenove motor controller hardware.
It receives Twist messages (linear and angular velocity) and translates
them into motor commands for the 4-wheel drive robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import os

# Add Freenove library path
sys.path.append(os.path.expanduser('~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server'))

try:
    from Motor import Motor
except ImportError as e:
    print(f"Error importing Freenove Motor library: {e}")
    print("Make sure Freenove code is installed at ~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/")
    sys.exit(1)


class MotorControlNode(Node):
    """
    ROS 2 Node that controls Freenove robot motors
    based on Twist velocity commands
    """
    
    def __init__(self):
        super().__init__('motor_control')
        
        # Initialize Freenove motor controller
        try:
            self.motor = Motor()
            self.get_logger().info('Freenove motor controller initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize motor controller: {e}')
            raise
        
        # Create subscriber to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/freenove/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Motor control parameters
        self.base_speed = 100  # Base motor power (0-100)
        self.turn_speed_factor = 0.5  # How much to reduce inside wheel speed when turning
        
        # Safety timeout - stop if no command received for this long
        self.timeout_duration = 1.0  # seconds
        self.last_cmd_time = self.get_clock().now()
        
        # Create timer to check for timeout
        self.timer = self.create_timer(0.1, self.timeout_check)
        
        self.get_logger().info('Motor control node started')
        self.get_logger().info('Subscribing to: /freenove/cmd_vel')
        self.get_logger().info('Send Twist messages to control the robot!')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands
        
        Twist message contains:
        - linear.x: Forward/backward velocity (m/s)
        - angular.z: Turning velocity (rad/s)
        """
        try:
            # Update last command time
            self.last_cmd_time = self.get_clock().now()
            
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Log received command (throttled to once per second)
            self.get_logger().info(
                f'Received cmd_vel: linear={linear_x:.3f}, angular={angular_z:.3f}',
                throttle_duration_sec=1.0
            )
            
            # Convert Twist command to motor speeds
            left_speed, right_speed = self.twist_to_motor_speeds(linear_x, angular_z)
            
            # Send commands to motors
            self.set_motor_speeds(left_speed, right_speed)
        
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {e}')
    
    def twist_to_motor_speeds(self, linear_x, angular_z):
        """
        Convert Twist velocities to differential drive motor speeds
        
        Differential drive kinematics:
        - Both wheels same speed + same direction = forward/backward
        - Wheels different speeds = turning
        - Wheels opposite directions = rotate in place
        
        Args:
            linear_x: Forward velocity (-1.0 to 1.0 normalized)
            angular_z: Turning velocity (-1.0 to 1.0 normalized)
        
        Returns:
            (left_speed, right_speed): Motor speeds from -100 to 100
        """
        # Normalize linear velocity to motor power
        forward_power = linear_x * self.base_speed
        
        # Normalize angular velocity to turning adjustment
        turn_adjustment = angular_z * self.base_speed * self.turn_speed_factor
        
        # Calculate differential drive speeds
        # Positive angular_z = turn left (slow down left wheels, speed up right)
        left_speed = forward_power - turn_adjustment
        right_speed = forward_power + turn_adjustment
        
        # Clamp to valid range
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        return int(left_speed), int(right_speed)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Send speed commands to physical motors
        
        Args:
            left_speed: Speed for left motors (-100 to 100)
            right_speed: Speed for right motors (-100 to 100)
        """
        try:
            # Freenove motor library:
            # setMotorModel(motor1, motor2, motor3, motor4)
            # For 4WD: motors 1,2 are left side, motors 3,4 are right side
            # Positive = forward, negative = backward
            
            self.motor.setMotorModel(left_speed, left_speed, right_speed, right_speed)
            
        except Exception as e:
            self.get_logger().error(f'Failed to set motor speeds: {e}')
    
    def stop_motors(self):
        """
        Emergency stop - set all motors to zero
        """
        try:
            self.motor.setMotorModel(0, 0, 0, 0)
            self.get_logger().info('Motors stopped')
        except Exception as e:
            self.get_logger().error(f'Failed to stop motors: {e}')
    
    def timeout_check(self):
        """
        Safety timeout check - stop motors if no recent commands
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.timeout_duration:
            # No command for too long - stop motors
            self.stop_motors()
    
    def destroy_node(self):
        """
        Clean up - stop motors when node shuts down
        """
        self.get_logger().info('Shutting down motor control node...')
        self.stop_motors()
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the motor control node
    
    Note: This node requires sudo/root privileges to access GPIO pins
    Run with: sudo -E ros2 run freenove_car motor_control_node
    """
    rclpy.init(args=args)
    
    try:
        motor_control = MotorControlNode()
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
