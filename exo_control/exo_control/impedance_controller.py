#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import copy

class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller')

        self.publisher = self.create_publisher(Float32MultiArray, 'torque_cmd', 10)
        self.publisher_ref = self.create_publisher(Float32MultiArray, 'x_ref', 10)
        self.subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        
        kp = 0.01
        kd = 0.01
        ki = 0.01

        self.Kp = [kp, kp]
        self.Kd = [kd, kd]
        self.Ki = [ki, ki]

        # Desired trajectory
        self.xd = [0.0, 0.0]
        self.vd = [0.0, 0.0]

        self.filtered_x = [0.0, 0.0]
        self.filtered_v = [0.0, 0.0]

        self.alpha = 0.1
        
        self.max_torque = 10.0
        self.prev_tau = [0.0, 0.0]
        self.max_tau_rate = 0.0

        self.error_integral = [0.0, 0.0]

        self.dt=0.005

        self.first = True

        self.get_logger().info("Impedance controller initialized.")

    def init_control_cycle(self,msg):
        self.xd = copy.deepcopy(msg.position)
        self.filtered_x = copy.deepcopy(msg.position)
        self.filtered_v = [0.0, 0.0]
        self.first = False

    def state_filter(self,msg):
        # Low-pass filter for x and v
        for i in range(2):
            self.filtered_x[i] = self.alpha * msg.position[i] + (1 - self.alpha) * self.filtered_x[i]
            self.filtered_v[i] = self.alpha * msg.velocity[i] + (1 - self.alpha) * self.filtered_v[i]

    def ramp_torque(self,raw_tau):
        # Torque ramping
        tau = [0.0, 0.0]
        for i in range(2):
            delta = raw_tau[i] - self.prev_tau[i]
            delta = max(min(delta, self.max_tau_rate), -self.max_tau_rate)
            tau[i] = self.prev_tau[i] + delta
            self.prev_tau[i] = tau[i]
        return tau

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            self.get_logger().warn("Received incomplete joint state.")
            return

        if self.first:
            self.init_control_cycle(msg)

        # filter positions and velocities
        self.state_filter(msg)

        self.error_integral[0] += (self.xd[0] - self.filtered_x[0]) * self.dt
        self.error_integral[1] += (self.xd[1] - self.filtered_x[1]) * self.dt
        
        # Raw torque calculation
        raw_tau = [
            # self.Kp[0] * (self.xd[0] - self.filtered_x[0]) + self.Kd[0] * (self.vd[0] - self.filtered_v[0]) + self.Ki[0] * self.error_integral[0],
            0.0,
            self.Kp[1] * (self.xd[1] - self.filtered_x[1]) + self.Kd[1] * (self.vd[1] - self.filtered_v[1]) + self.Ki[1] * self.error_integral[1],
        ]

        tau = raw_tau

        # tau = self.ramp_torque(raw_tau)

        # Optional torque saturation
        # tau = [max(min(t, self.max_torque), -self.max_torque) for t in tau]

        # self.get_logger().info(f"Torque: {tau[1]:.2f} | x: {self.xd[1]:.2f}, {self.filtered_x[1]:.2f}")

        # Publish torque command
        torque_msg = Float32MultiArray()
        torque_msg.data = tau
        self.publisher.publish(torque_msg)
        ref_x_msg = Float32MultiArray()
        ref_x_msg.data = self.xd
        self.publisher_ref.publish(ref_x_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down impedance controller.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
