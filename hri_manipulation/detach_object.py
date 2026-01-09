import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject


class AttachObject(Node):
    def __init__(self):
        super().__init__("attach_object")
        self.client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_logger().info("Waiting for ApplyPlanningScene service...")
        self.client.wait_for_service()
        self.get_logger().info("Connected ✅")

    def attach(self):
        aco = AttachedCollisionObject()
        aco.link_name = "tool0"
        aco.object.id = "box"
        aco.object.operation = CollisionObject.REMOVE

        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.is_diff = True

        self.client.call_async(req)
        self.get_logger().info("📎 Box DETACHED from tool0")


def main():
    rclpy.init()
    node = AttachObject()
    node.attach()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
