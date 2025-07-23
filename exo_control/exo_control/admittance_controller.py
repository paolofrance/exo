#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import copy

class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        self.position_pub = self.create_publisher(Float32MultiArray, 'position_cmd', 10)
        self.position_filt_pub = self.create_publisher(Float32MultiArray, 'position_filtered', 10)
        self.est_torque_pub = self.create_publisher(Float32MultiArray, 'torque_estimated', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.external_torque_sub = self.create_subscription(Float32MultiArray, 'external_torque', self.external_torque_callback, 10)


        m=1.0
        c=0.50
        c_ratio = 0.8
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

        # External torque input (initialized to zero)
        self.external_tau = [0.0, 0.0]

        self.alpha = 0.1

        self.init_positions =True

        self.get_logger().info("Admittance controller initialized.")
    
    def apply_deadband_and_saturation(self, values, deadband=0.5, limit=5.0):
        result = []
        for v in values:
            if abs(v) < deadband:
                v_out = 0.0
            else:
                if v > 0:
                    v_out = (v - deadband) / (limit - deadband) * limit
                else:
                    v_out = (v + deadband) / (limit - deadband) * limit
                v_out = max(min(v_out, limit), -limit)
            result.append(v_out)
        return result


    def state_filter(self,msg: JointState):
        for i in range(2):
            self.filtered_p[i] = self.alpha * msg.position[i] + (1 - self.alpha) * self.filtered_p[i]
            self.filtered_v[i] = self.alpha * msg.velocity[i] + (1 - self.alpha) * self.filtered_v[i]
            self.filtered_e[i] = self.alpha * msg.effort  [i] + (1 - self.alpha) * self.filtered_e[i]
            
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
            self.x_ref = list(msg.position[:2])
            self.filtered_p = list(msg.position[:2])
            self.filtered_v = list(msg.velocity[:2])
            self.filtered_e = list(msg.effort[:2])
            self.init_positions = False

        self.state_filter(msg)
        self.filtered_e = self.apply_deadband_and_saturation(self.filtered_e, deadband=0.1, limit=10.0)

        self.measured_pos = list(msg.position[:2])
        self.measured_vel = list(msg.velocity[:2])
        self.measured_eff = list(msg.effort  [:2])

        # Admittance dynamics integration for each joint:
        for i in range(2):
            self.a[i] = (self.filtered_e[i] - self.C[i] * self.v[i] - self.K[i] * (self.x[i] - self.x_ref[i])) / self.M[i]
            # self.a[i] = (self.filtered_e[i] - self.C[i] * self.v[i] - self.K[i] * (self.filtered_p[i] - self.x_ref[i])) / self.M[i]
            self.v[i] += self.a[i] * self.dt
            self.x[i] += self.v[i] * self.dt

        # Publish desired position commands
        pos_msg = Float32MultiArray()
        pos_msg.data = [-self.x[1],self.x[1]]
        self.position_pub.publish(pos_msg)
        
        # pos_msg_f = Float32MultiArray()
        # pos_msg_f.data = self.filtered_p
        # self.position_filt_pub.publish(pos_msg_f)

        est_torque_msg = Float32MultiArray()
        est_torque_msg.data = self.filtered_e
        self.est_torque_pub.publish(est_torque_msg)


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
