#!/usr/bin/env python3
"""Sim stand-in for qcar_hardware_interface.py's command path.

On the real car, /cmd_vel never reaches the wheels directly: the hardware
interface turns it into a motor command [throttle, steering] and sends that
at 50 Hz. This node reproduces that step with the same constants and the same
order of operations, then turns the result back into the Twist that Gazebo's
AckermannSteering plugin expects on /cmd_vel_hw:

    /cmd_vel -> cmd_vel_callback()  -> throttle, target_steering
             -> control_loop() 50Hz -> timeout, steering lag -> [throttle, steering]
             -> Twist(v = throttle * throttle_to_speed, w = v * tan(steering) / L)

AckermannSteering inverts w = v*tan(delta)/L with the same wheel_base (0.256),
so the simulated steering angle equals the one the real car would be sent.

throttle_to_speed maps PWM duty to m/s. The default (max_speed / 0.2 = 2.5)
makes throttle an exact inverse of the hardware's v -> throttle mapping, i.e.
the sim car drives at the commanded speed. The real motor's duty -> speed
curve is not known here; calibrate it from OptiTrack runs and set this
parameter to close that part of the sim-to-real gap.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist


class QCarHwEmulator(Node):
    def __init__(self):
        super().__init__('qcar_hw_emulator')

        # Same parameters and defaults as qcar_hardware_interface.py; the
        # run_qcar_*.sh scripts pass max_speed:=0.5 max_steering_angle:=0.5.
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('throttle_to_speed', 0.5 / 0.2)
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.throttle_to_speed = self.get_parameter('throttle_to_speed').value

        self.wheel_base = 0.256

        self.current_throttle = 0.0
        self.target_steering = 0.0
        self.current_steering = 0.0
        self.steering_tau = 0.16

        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = Duration(seconds=1.0)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.hw_pub = self.create_publisher(Twist, '/cmd_vel_hw', 10)

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            'QCar hardware emulator: /cmd_vel -> /cmd_vel_hw (throttle_to_speed={:.3f})'.format(
                self.throttle_to_speed))

    # --- verbatim logic from qcar_hardware_interface.py ---------------------

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Throttle command
        self.current_throttle = np.clip(linear_vel / self.max_speed * 0.2, -0.2, 0.2)

        # Steering command
        steering_angle_rad = self.compute_steering_from_angular_velocity(linear_vel, angular_vel)
        self.target_steering = np.clip(steering_angle_rad, -0.5, 0.5)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = 0.02

        time_since_cmd = current_time - self.last_cmd_time
        if time_since_cmd > self.cmd_timeout:
            self.current_throttle = 0.0
            self.target_steering = 0.0

        # Steering Lag Filter
        alpha = dt / (self.steering_tau + dt)
        self.current_steering = (alpha * self.target_steering) + ((1.0 - alpha) * self.current_steering)

        mtr_cmd = np.array([self.current_throttle, self.current_steering])
        self.write_motor_command(mtr_cmd)

    def compute_steering_from_angular_velocity(self, linear_velocity, desired_angular_velocity):
        if abs(linear_velocity) < 0.05:
            if abs(desired_angular_velocity) > 0.01:
                return np.sign(desired_angular_velocity) * 0.5
            else:
                return 0.0
        try:
            steering_angle = math.atan((self.wheel_base * desired_angular_velocity) / linear_velocity)
            return steering_angle
        except Exception:
            return 0.0

    # --- sim replacement for qcar.read_write_std(mtr_cmd, LEDs) -------------

    def write_motor_command(self, mtr_cmd):
        throttle, steering = float(mtr_cmd[0]), float(mtr_cmd[1])
        speed = throttle * self.throttle_to_speed

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed * math.tan(steering) / self.wheel_base
        self.hw_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = QCarHwEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
