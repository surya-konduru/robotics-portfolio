#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PID:

    def __init__(self, kp, ki, kd):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error):

        self.integral += error

        derivative = error - self.previous_error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.previous_error = error

        return output


class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.wheel_radius = 0.05
        self.left_pid = PID(1.0, 0.01, 0.1)
        self.right_pid = PID(1.0, 0.01, 0.1)


    def cmd_callback(self, msg):

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        wheel_separation = 0.30
        wheel_radius = 0.05

        left_wheel_speed = (
            linear_velocity -
            angular_velocity * wheel_separation / 2
        ) / wheel_radius

        right_wheel_speed = (
            linear_velocity +
            angular_velocity * wheel_separation / 2
        ) / wheel_radius

        actual_left_speed = 0.0
        actual_right_speed = 0.0

        left_error = left_wheel_speed - actual_left_speed
        right_error = right_wheel_speed - actual_right_speed

        left_control = self.left_pid.compute(left_error)
        right_control = self.right_pid.compute(right_error)

        self.get_logger().info(
            f'Linear Velocity : {linear_velocity:.2f} m/s'
        )

        self.get_logger().info(
            f'Angular Velocity: {angular_velocity:.2f} rad/s'
        )

        self.get_logger().info(
            f'Left Wheel Speed : {left_wheel_speed:.2f} rad/s'
        )

        self.get_logger().info(
            f'Right Wheel Speed: {right_wheel_speed:.2f} rad/s'
        )
        self.get_logger().info(
            f'Left PID Output : {left_control:.2f}'
        )
        self.get_logger().info(
            f'Right PID Output: {right_control:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DiffDriveController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()