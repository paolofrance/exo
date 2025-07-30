#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import copy
import time
from std_msgs.msg import Float32MultiArray
import math
import numpy as np


class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        self.position_pub = self.create_publisher(Float32MultiArray, 'position_cmd', 10)
        self.position_filt_pub = self.create_publisher(Float32MultiArray, 'position_filtered', 10)
        self.est_torque_pub = self.create_publisher(Float32MultiArray, 'torque_estimated', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.external_torque_sub = self.create_subscription(Float32MultiArray, 'external_torque', self.external_torque_callback, 10)

        m=2.5
        c=1.50
        c_ratio = .9
        k=0.0

        if k != 0.0 and m != 0.0:
            c = 2 * c_ratio * (m * k) ** 0.5

        self.M = [m,m]      # virtual inertia [kg*m^2]
        self.C = [c,c]   # virtual damping [Nms/rad]
        self.K = [k,k]  # virtual stiffness [Nm/rad]

        self.dt = 0.005

        # States for each joint
        self.x = [0.0, 0.0]         # position
        self.v = [0.0, 0.0]         # velocity
        self.a = [0.0, 0.0]         # acceleration

        self.filtered_p = [0.0, 0.0]         # position
        self.filtered_v = [0.0, 0.0]         # velocity
        self.filtered_e = [0.0, 0.0]         # position

        # Desired trajectory (initially zeros)
        self.x_ref = [0.0, 0.0]

        # Latest measured joint states
        self.measured_pos = [0.0, 0.0]
        self.measured_vel = [0.0, 0.0]
        self.measured_eff = [0.0, 0.0]
        self.measured_eff_zero = [0.0, 0.0]

        # External torque input (initialized to zero)
        self.external_tau = [0.0, 0.0]

        self.alpha = 0.1
        self.alpha_eff = 0.10

        self.init_positions =True

        self.get_logger().info("Admittance controller initialized.")

    def initial_soft_sync(self, x, max_sync_time=3.0, sync_rate=100.0, threshold=0.01, alpha=0.05):

        self.get_logger().info("Initial motors synchronization")
        dt = 1.0 / sync_rate
        start_time = time.time()

        self.x = [x[0], x[1]]  # initialize if needed

        while abs(self.x[0] + self.x[1]) > threshold and (time.time() - start_time) < max_sync_time:
            # Smoothly adjust x[0] to approach -x[1]
            self.x[0] = (1 - alpha) * self.x[0] + alpha * (-self.x[1])

            # Publish positions
            pos_msg = Float32MultiArray()
            pos_msg.data = self.x
            self.position_pub.publish(pos_msg)

            time.sleep(dt)

        # Final snap to constraint
        self.x[0] = -self.x[1]
        pos_msg = Float32MultiArray()
        pos_msg.data = self.x
        self.position_pub.publish(pos_msg)

        self.get_logger().info("Initial soft sync complete")

    
    def apply_deadband_and_saturation(self, values, deadband=0.5, limit=5.0):
        result = []
        for v in values:
            if abs(v) < deadband:
                v_out = 0.0
            else:
                if v > 0:
                    v_out = (v - deadband)# / (limit - deadband) * limit
                else:
                    v_out = (v + deadband)# / (limit - deadband) * limit
                # v_out = max(min(v_out, limit), -limit)
            result.append(v_out)
        return result


    def state_filter(self, msg: JointState):
        for i in range(2):
            # Subtract initial torque offset to zero the reading
            zeroed_effort = msg.effort[i] - self.measured_eff_zero[i]

            self.filtered_p[i] = self.alpha * msg.position[i] + (1 - self.alpha) * self.filtered_p[i]
            self.filtered_v[i] = self.alpha * msg.velocity[i] + (1 - self.alpha) * self.filtered_v[i]
            self.filtered_e[i] = self.alpha_eff * zeroed_effort + (1 - self.alpha_eff) * self.filtered_e[i]

            
    def external_torque_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("External torque message incomplete.")
            return
        self.external_tau = list(msg.data)

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 2 or len(msg.velocity) < 2:
            self.get_logger().warn("Joint state message incomplete.")
            return
        
        if self.init_positions:
            
            self.x = list(msg.position[:2])

            self.initial_soft_sync(self.x,alpha=0.05)

            self.x_ref = list(msg.position[:2])
            self.filtered_p = list(msg.position[:2])
            self.filtered_v = list(msg.velocity[:2])
            self.filtered_e = list(msg.effort[:2])
            self.measured_eff_zero = list(msg.effort[:2])
            self.init_positions = False

        self.state_filter(msg)
        # self.filtered_e = self.apply_deadband_and_saturation(self.filtered_e, deadband=0.1, limit=10.0)

        self.measured_pos = list(msg.position[:2])
        self.measured_vel = list(msg.velocity[:2])
        self.measured_eff = list(msg.effort  [:2])
        
        # Admittance dynamics integration for each joint:

        tau_ass = [0.0, 0.0]
        for i in range(2):
            tau_ass[i] = self.filtered_e[i] * 0.0
            self.a[i] = ( tau_ass[i] + self.filtered_e[i] - self.C[i] * self.v[i] - self.K[i] * (self.x[i] - self.x_ref[i])) / self.M[i]
            self.v[i] += self.a[i] * self.dt
            self.x[i] += self.v[i] * self.dt

        theta_ref_l = copy.deepcopy(-self.x[1] )
        theta_ref_r = copy.deepcopy( self.x[1] )

        # theta_ref_l = np.clip(-self.x[1], 0.0, 1.0)
        # theta_ref_r = np.clip( self.x[1], -1.0, 0.0)

        # theta_ref_l = self.soft_clip(-self.x[1], 0.0, 1.0, softness=0.2)
        # theta_ref_r = self.soft_clip( self.x[1], -1.0, 0.0, softness=0.2)

        # Publish desired position commands
        pos_msg = Float32MultiArray()
        pos_msg.data = [theta_ref_l,theta_ref_r]
        # pos_msg.data = [-self.x[1],self.x[1]]
        self.position_pub.publish(pos_msg)
        
        est_torque_msg = Float32MultiArray()
        est_torque_msg.data = self.filtered_e
        self.est_torque_pub.publish(est_torque_msg)

    def soft_clip(self,x, lower_limit, upper_limit, softness=0.1):
        midpoint = 0.5 * (upper_limit + lower_limit)
        range_half = 0.5 * (upper_limit - lower_limit)
        return midpoint + range_half * np.tanh((x - midpoint) / (range_half * softness))


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down admittance controller.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
