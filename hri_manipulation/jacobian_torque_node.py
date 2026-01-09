import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from .planar_arm import forward_kinematics

# Using same link lengths as planar_arm.py
L1 = 0.4
L2 = 0.3
L3 = 0.2

def jacobian(q):
    q1, q2, q3 = q
    a1 = q1
    a2 = q1 + q2
    a3 = q1 + q2 + q3

    J = np.zeros((3, 3))

    # dx/dq
    J[0,0] = -L1*np.sin(a1) - L2*np.sin(a2) - L3*np.sin(a3)
    J[0,1] = -L2*np.sin(a2) - L3*np.sin(a3)
    J[0,2] = -L3*np.sin(a3)

    # dy/dq
    J[1,0] = L1*np.cos(a1) + L2*np.cos(a2) + L3*np.cos(a3)
    J[1,1] = L2*np.cos(a2) + L3*np.cos(a3)
    J[1,2] = L3*np.cos(a3)

    # d(alpha)/dq
    J[2,:] = [1, 1, 1]

    return J


class JacobianTorqueNode(Node):
    def __init__(self):
        super().__init__('jacobian_torque_node')

        # Subscribe to joint angles from IK node
        self.sub_q = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.callback_q,
            10
        )

        # Publish torque output
        self.pub_tau = self.create_publisher(
            Float64MultiArray,
            'joint_torque',
            10
        )

        # Simulated force (constant for now)
        self.F = np.array([2.0, 1.0, 0.5])  # Fx, Fy, F_alpha

        self.q = None

        self.get_logger().info("Jacobian Torque Node started")

    def callback_q(self, msg):
        self.q = np.array(msg.data)

        J = jacobian(self.q)

        tau = J.T @ self.F  # torque = J^T * Force

        msg_tau = Float64MultiArray()
        msg_tau.data = tau.tolist()

        self.pub_tau.publish(msg_tau)
        self.get_logger().info(f"Torque output: {tau}")


def main(args=None):
    rclpy.init(args=args)
    node = JacobianTorqueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
