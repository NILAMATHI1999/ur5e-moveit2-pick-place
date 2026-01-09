import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np


class AdmittanceNode(Node):
    def __init__(self):
        super().__init__('admittance_node')

        # Subscribe to torque inputs
        self.sub_tau = self.create_subscription(
            Float64MultiArray,
            'joint_torque',
            self.callback_tau,
            10
        )

        # Publish simulated EE position
        self.pub_pos = self.create_publisher(
            Float64MultiArray,
            'admittance_position',
            10
        )

        # Admittance parameters
        self.M = 1.0    # Mass
        self.D = 4.0    # Damping
        self.K = 10.0   # Stiffness

        # State variables
        self.x = 0.0     # Position
        self.v = 0.0     # Velocity

        # Force is None until torque is received
        self.F = None

        # Timer for integration (every 10 ms)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.integrate)

        self.get_logger().info("Admittance Node started")

    def callback_tau(self, msg):
        tau = np.array(msg.data)

        # Use torque magnitude as force input
        self.F = np.linalg.norm(tau)

        # Debug information
        self.get_logger().info(f"Received torque: {tau}")
        self.get_logger().info(f"Computed force magnitude F: {self.F}")

    def integrate(self):
        # Wait until first torque arrives
        if self.F is None:
            return

        # Compute acceleration: M x'' + D x' + Kx = F
        a = (self.F - self.D * self.v - self.K * self.x) / self.M

        # Integrate
        self.v += a * self.dt
        self.x += self.v * self.dt

        # Publish
        msg = Float64MultiArray()
        msg.data = [float(self.x)]
        self.pub_pos.publish(msg)

        self.get_logger().info(f"Admittance x = {self.x:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

