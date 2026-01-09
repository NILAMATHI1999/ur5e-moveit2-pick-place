import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


class PickPlaceFSM(Node):
    """
    Minimal pick-place-home sequencer.
    Publishes a 2D target pose (x, y, theta) that your IK node can subscribe to.
    """
    def __init__(self):
        super().__init__('pick_place_fsm')
        self.pub = self.create_publisher(Pose2D, '/target_pose', 10)

        # Simple 2D targets for your planar IK demo
        self.targets = [
            ('home',  0.30, 0.10, 0.0),
            ('pick',  0.25, 0.20, 0.0),
            ('place', 0.35, 0.20, 0.0),
        ]
        self.i = 0
        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info("PickPlaceFSM started. Publishing /target_pose every 2s.")

    def tick(self):
        name, x, y, th = self.targets[self.i]
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(th)
        self.pub.publish(msg)
        self.get_logger().info(f"Sent target: {name} -> x={x:.2f}, y={y:.2f}, th={th:.2f}")
        self.i = (self.i + 1) % len(self.targets)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
