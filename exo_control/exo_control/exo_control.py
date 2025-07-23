#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from odrive.enums import CONTROL_MODE_TORQUE_CONTROL, CONTROL_MODE_POSITION_CONTROL
from odrive.enums import INPUT_MODE_PASSTHROUGH
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
import math

MODE_NONE = 0 
MODE_POSITION = CONTROL_MODE_POSITION_CONTROL
MODE_TORQUE = CONTROL_MODE_TORQUE_CONTROL

import odrive
import time
import signal
import sys

from exo_interfaces.srv import SetControlMode

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
        self.position_sub = self.create_subscription(Float32MultiArray, 'position_cmd', self.position_callback, 10)
        self.mode_srv = self.create_service(SetControlMode, 'set_control_mode', self.set_control_mode)

        self.get_logger().info("Clearing errors...")
        self.odrive.clear_errors()

        for axis in [self.axis0, self.axis1]:
            if not axis.motor.is_calibrated:
                self.get_logger().error("Axis not calibrated.")
                if not axis.encoder.is_ready:
                    self.get_logger().error("encoder not ready.")
                    return

        self.current_mode = MODE_POSITION
        
        self.set_odrive_mode(self.current_mode)

        self.torque_0 = 0.0
        self.torque_1 = 0.0
        self.pos_0 = 0.0
        self.pos_1 = 0.0

        self.GEAR_RATIO = 30.0

        self.positions_initialized = False

        self.create_timer(0.005, self.publish_joint_states)
        self.create_timer(1.0, self.monitor_errors)
        self.create_timer(0.005, self.send_commands)

        signal.signal(signal.SIGINT, self.shutdown_handler)

    def set_odrive_mode(self, mode_int):
        if mode_int not in [MODE_TORQUE, MODE_POSITION, MODE_NONE]:
            self.get_logger().error(f"Invalid control mode int: {mode_int}")
            return False

        self.current_mode = mode_int

        for axis in [self.axis0, self.axis1]:
            if mode_int == MODE_NONE:
                axis.requested_state = AXIS_STATE_IDLE
            else:
                axis.controller.config.control_mode = mode_int
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        mode_str = {MODE_NONE: "NONE", MODE_TORQUE: "TORQUE", MODE_POSITION: "POSITION"}[mode_int]
        self.get_logger().info(f"Set control mode to {mode_str}")
        return True


    def set_control_mode(self, request, response):
        mode_int = request.mode
        success = self.set_odrive_mode(mode_int)

        response.success = success
        if success:
            response.message = f"Control mode set to {mode_int}"
        else:
            response.message = f"Failed to set control mode: {mode_int}"
        return response

    def torque_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 torque values, got: {len(msg.data)}")
            return
        self.torque_0, self.torque_1 = msg.data

    def position_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 position values, got: {len(msg.data)}")
            return
        self.pos_0, self.pos_1 = msg.data

    def send_commands(self):
        if self.current_mode == MODE_NONE:
            return  # No commands sent in passive mode
        elif self.current_mode == MODE_TORQUE:
            self.axis0.controller.input_torque = self.torque_0
            self.axis1.controller.input_torque = self.torque_1
        elif self.current_mode == MODE_POSITION:
            if not self.positions_initialized:
                self.pos_0 = self.axis0.encoder.pos_estimate * 2 * math.pi / self.GEAR_RATIO
                self.pos_1 = self.axis1.encoder.pos_estimate * 2 * math.pi / self.GEAR_RATIO
                self.positions_initialized = True
            self.axis0.controller.input_pos = self.pos_0 / (2 * math.pi) * self.GEAR_RATIO
            self.axis1.controller.input_pos = self.pos_1 / (2 * math.pi) * self.GEAR_RATIO


    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            self.axis0.encoder.pos_estimate * 2 * math.pi / self.GEAR_RATIO,
            self.axis1.encoder.pos_estimate * 2 * math.pi / self.GEAR_RATIO
        ]
        msg.velocity = [
            self.axis0.encoder.vel_estimate * 2 * math.pi / self.GEAR_RATIO,
            self.axis1.encoder.vel_estimate * 2 * math.pi / self.GEAR_RATIO
        ]
        msg.effort = [
            self.axis0.motor.current_control.Iq_measured * 0.0250606 * self.GEAR_RATIO,
            self.axis1.motor.current_control.Iq_measured * 0.0250606 * self.GEAR_RATIO
        ]
        self.joint_state_pub.publish(msg)

    def monitor_errors(self):
        self.get_logger().info(f"current control mode: {self.current_mode}")
        for i, axis in enumerate([self.axis0, self.axis1]):
            if axis.motor.error or axis.encoder.error or axis.controller.error:
                self.get_logger().error(
                    f"Axis {i} errors - Motor: {axis.motor.error}, Encoder: {axis.encoder.error}, Controller: {axis.controller.error}"
                )

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
