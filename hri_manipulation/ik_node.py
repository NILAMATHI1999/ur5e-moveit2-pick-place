import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from .planar_arm import inverse_kinematics

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # Publisher: joint angles [q1, q2, q3]
        self.pub = self.create_publisher(Float64MultiArray, 'joint_angles', 10)

        # Timer: compute IK every 1 second
        self.timer = self.create_timer(1.0, self.compute_and_publish)

        self.get_logger().info("IK Node started")

    def compute_and_publish(self):
        # Example target point (you can change)
        x_target = 0.5
        y_target = 0.2
        alpha_target = np.deg2rad(10)   # orientation

        q = inverse_kinematics(x_target, y_target, alpha_target)

        msg = Float64MultiArray()
        msg.data = q.tolist()

        self.pub.publish(msg)
        self.get_logger().info(f"Published IK solution: {q}")

def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
