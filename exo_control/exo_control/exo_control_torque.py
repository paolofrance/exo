#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

import odrive
from odrive.enums import *
import time
import signal
import sys

class ODriveControllerNode(Node):
    def __init__(self):
        super().__init__('odrive_controller_node')

        self.get_logger().info("Connecting to ODrive...")
        self.odrive = odrive.find_any()
        self.axis0 = self.odrive.axis0
        self.axis1 = self.odrive.axis1
        self.get_logger().info("ODrive connected.")

        self.joint_names = ['joint_0', 'joint_1']

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.torque_sub = self.create_subscription(Float32MultiArray, 'torque_cmd', self.torque_callback, 10)

        self.get_logger().info("Clearing errors...")
        self.odrive.clear_errors()
        for axis in [self.axis0, self.axis1]:
            # Set control and input mode for torque control
            axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

            # Set motor current limits (adjust accordingly)
            # axis.motor.config.current_lim = 10.0
            
            # Ensure motor and encoder are calibrated before closed-loop
            if not axis.motor.is_calibrated or not axis.encoder.is_ready:
                self.get_logger().error("Axis not calibrated or encoder not ready, calibrate first!")
                # Optionally, run calibration here or exit
                # self.calibrate_axis(axis)
                return

            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.torque_0 = 0.0
        self.torque_1 = 0.0

        # Timers
        self.create_timer(0.005, self.publish_joint_states)
        self.create_timer(1.0, self.monitor_errors)
        self.create_timer(0.005, self.send_commands)
        # Register shutdown handler
        signal.signal(signal.SIGINT, self.shutdown_handler)
    
    def calibrate_axis(self, axis):
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        axis.motor.config.pre_calibrated = True
        axis.encoder.config.pre_calibrated = True
        axis.requested_state = AXIS_STATE_IDLE
        self.odrive.save_configuration()
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            self.axis0.encoder.pos_estimate,
            self.axis1.encoder.pos_estimate
        ]
        msg.velocity = [
            self.axis0.encoder.vel_estimate,
            self.axis1.encoder.vel_estimate
        ]
        msg.effort = [
            self.axis0.motor.current_control.Iq_measured,
            self.axis1.motor.current_control.Iq_measured
        ]
        self.joint_state_pub.publish(msg)

    def send_commands(self):
        self.axis0.controller.input_torque = self.torque_0
        self.axis1.controller.input_torque = self.torque_1
        

    def torque_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 torque values, got: {len(msg.data)}")
            return
        torque_0, torque_1 = msg.data
        self.torque_0 = torque_0
        self.torque_1 = torque_1
        self.get_logger().debug(f"Applied torques: {torque_0}, {torque_1}")

    def monitor_errors(self):
        for i, axis in enumerate([self.axis0, self.axis1]):
            motor_error = axis.motor.error
            encoder_error = axis.encoder.error
            controller_error = axis.controller.error
            if motor_error != 0 or encoder_error != 0 or controller_error != 0:
                self.get_logger().error(f"Axis {i} errors - Motor: {motor_error}, Encoder: {encoder_error}, Controller: {controller_error}")

    def shutdown_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected! Stopping ODrive...")
        for i, axis in enumerate([self.axis0, self.axis1]):
            axis.requested_state = AXIS_STATE_IDLE
            self.get_logger().info(f"Axis {i} set to IDLE.")
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_handler(None, None)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
