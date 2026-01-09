import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene


class AddObject(Node):
    def __init__(self):
        super().__init__("add_object")
        self.client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_logger().info("Waiting for ApplyPlanningScene service...")
        self.client.wait_for_service()
        self.get_logger().info("Connected ✅")

    def add_box(self):
        box = CollisionObject()
        box.id = "box"
        box.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.10]  # 5x5x10 cm

        pose = Pose()
        pose.position.x = 0.45
        pose.position.y = 0.0
        pose.position.z = 0.10  # sits on table
        pose.orientation.w = 1.0

        box.primitives.append(primitive)
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(box)
        req.scene.is_diff = True

        self.client.call_async(req)
        self.get_logger().info("📦 Box added to planning scene")


def main():
    rclpy.init()
    node = AddObject()
    node.add_box()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
