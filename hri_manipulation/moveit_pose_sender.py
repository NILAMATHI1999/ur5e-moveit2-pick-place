#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from hri_manipulation.gripper import Gripper

SUCCESS = 1  # MoveIt success code


class PoseSender(Node):
    def __init__(self):
        super().__init__("pose_sender")

        # ====== DEMO TUNING (stable + visible) ======
        self.VEL = 0.10
        self.ACC = 0.10

        # Heights (reviewer-visible)
        self.PREPICK_Z = 0.20
        self.GRASP_Z = 0.06
        self.LIFT_Z = 0.30

        self.PREPLACE_Z = 0.20
        self.PLACE_Z = 0.06
        self.RETREAT_Z = 0.30

        self.PLACE_X = 0.45
        self.PLACE_Y = 0.20

        # Planning robustness
        self.PLANNER_ID = "RRTConnectkConfigDefault"
        self.ALLOWED_PLANNING_TIME = 12.0
        self.NUM_PLANNING_ATTEMPTS = 10

        # Relaxed constraints (THIS reduces 99999/-2)
        self.POS_TOL = 0.08  # meters (box side lengths used below)
        self.ORI_TOL = 0.35  # radians (looser than 0.2)
        # ===========================================

        # MoveIt2 action client
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for /move_action action server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected to /move_action ✅")

        # Planning scene service
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_logger().info("Waiting for /apply_planning_scene...")
        self.scene_client.wait_for_service()
        self.get_logger().info("Connected to /apply_planning_scene ✅")

        # Object pose subscriber
        self.latest_object_pose: PoseStamped | None = None
        self.create_subscription(PoseStamped, "/object_pose", self.object_pose_cb, 10)

    def object_pose_cb(self, msg: PoseStamped):
        self.latest_object_pose = msg

    # ---------------- MoveIt goal helpers ----------------
    def _build_goal(self, x, y, z, qx, qy, qz, qw) -> MoveGroup.Goal:
        target = PoseStamped()
        target.header.frame_id = "world"
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)

        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)

        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = "ur_manipulator"
        req.num_planning_attempts = int(self.NUM_PLANNING_ATTEMPTS)
        req.allowed_planning_time = float(self.ALLOWED_PLANNING_TIME)
        req.max_velocity_scaling_factor = float(self.VEL)
        req.max_acceleration_scaling_factor = float(self.ACC)

        # Encourage a stable, fast planner for demo
        req.pipeline_id = "ompl"
        req.planner_id = self.PLANNER_ID

        # ---- tool0 constraints (relaxed to avoid failures) ----
        pc = PositionConstraint()
        pc.header = target.header
        pc.link_name = "tool0"

        tol_box = SolidPrimitive()
        tol_box.type = SolidPrimitive.BOX
        tol_box.dimensions = [self.POS_TOL, self.POS_TOL, self.POS_TOL]
        pc.constraint_region.primitives.append(tol_box)
        pc.constraint_region.primitive_poses.append(target.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = target.header
        oc.link_name = "tool0"
        oc.orientation = target.pose.orientation
        oc.absolute_x_axis_tolerance = float(self.ORI_TOL)
        oc.absolute_y_axis_tolerance = float(self.ORI_TOL)
        oc.absolute_z_axis_tolerance = float(self.ORI_TOL)
        oc.weight = 0.5  # lower weight makes it easier to solve than pos

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints = [c]
        # ------------------------------------------------------

        return goal

    def _send_goal_and_wait(self, goal: MoveGroup.Goal) -> int:
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            return -1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val

    def go(self, x, y, z, qx, qy, qz, qw, label="") -> bool:
        self.get_logger().info(f"➡️ {label}  x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Clean retry: only “warn” if ALL attempts fail
        for attempt in (1, 2, 3):
            goal = self._build_goal(x, y, z, qx, qy, qz, qw)

            # Slightly more time on later attempts
            if attempt == 2:
                goal.request.allowed_planning_time = max(goal.request.allowed_planning_time, 18.0)
            if attempt == 3:
                goal.request.allowed_planning_time = max(goal.request.allowed_planning_time, 25.0)
                goal.request.num_planning_attempts = max(goal.request.num_planning_attempts, 25)

            code = self._send_goal_and_wait(goal)
            if code == SUCCESS:
                return True

            # Keep logs clean: info on retry, not scary warn
            self.get_logger().info(f"   (planning retry {attempt}/3, code={code})")

        self.get_logger().warn(f"❌ Failed to plan for step: {label}")
        return False

    # ---------------- Object pose helpers ----------------
    def wait_for_object_pose(self, timeout_sec: float = 10.0) -> bool:
        start = time.time()
        while rclpy.ok() and self.latest_object_pose is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_object_pose is not None

    def get_object_xy(self):
        # fallback
        x, y = 0.45, 0.00
        if self.latest_object_pose is not None:
            x = self.latest_object_pose.pose.position.x
            y = self.latest_object_pose.pose.position.y
        return x, y

    # ---------------- Planning scene: attach / detach ----------------
    def _apply_scene(self, aco: AttachedCollisionObject, label: str) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.is_diff = True

        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        ok = bool(resp and resp.success)
        self.get_logger().info(f"{label} {'✅' if ok else '❌'}")
        return ok

    def attach_box(self) -> bool:
        aco = AttachedCollisionObject()
        aco.link_name = "tool0"
        aco.object.id = "box"
        aco.object.operation = CollisionObject.ADD
        return self._apply_scene(aco, "📎 Attached box (planning scene)")

    def detach_box(self) -> bool:
        aco = AttachedCollisionObject()
        aco.link_name = "tool0"
        aco.object.id = "box"
        aco.object.operation = CollisionObject.REMOVE
        return self._apply_scene(aco, "🧷 Detached box (planning scene)")


def main(args=None):
    rclpy.init(args=args)
    node = PoseSender()
    gripper = Gripper(node.get_logger())

    # Tool-down-ish orientation
    qx, qy, qz, qw = -0.707, 0.0, 0.0, 0.707

    try:
        ok = node.wait_for_object_pose(timeout_sec=10.0)
        if not ok:
            node.get_logger().info("No /object_pose in 10s -> using fallback x,y")

        obj_x, obj_y = node.get_object_xy()

        # HOME
        gripper.open()
        if not node.go(0.30, 0.00, 0.40, qx, qy, qz, qw, label="HOME"):
            return
        time.sleep(0.4)

        # PRE-PICK
        if not node.go(obj_x, obj_y, node.PREPICK_Z, qx, qy, qz, qw, label="PRE-PICK"):
            return
        time.sleep(0.4)

        # DESCEND
        if not node.go(obj_x, obj_y, node.GRASP_Z, qx, qy, qz, qw, label="DESCEND"):
            return
        time.sleep(0.4)

        # GRASP + ATTACH
        gripper.close()
        time.sleep(0.3)
        node.attach_box()
        time.sleep(0.4)

        # LIFT (visible)
        if not node.go(obj_x, obj_y, node.LIFT_Z, qx, qy, qz, qw, label="LIFT"):
            return
        time.sleep(0.4)

        # PRE-PLACE
        if not node.go(node.PLACE_X, node.PLACE_Y, node.PREPLACE_Z, qx, qy, qz, qw, label="PRE-PLACE"):
            return
        time.sleep(0.4)

        # PLACE
        if not node.go(node.PLACE_X, node.PLACE_Y, node.PLACE_Z, qx, qy, qz, qw, label="PLACE"):
            return
        time.sleep(0.3)

        # RELEASE + DETACH
        gripper.open()
        time.sleep(0.3)
        node.detach_box()
        time.sleep(0.3)

        # RETREAT
        node.go(node.PLACE_X, node.PLACE_Y, node.RETREAT_Z, qx, qy, qz, qw, label="RETREAT")

        node.get_logger().info("✅ Pick & Place complete (demo-clean).")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
