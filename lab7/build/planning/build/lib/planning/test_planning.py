import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class Planner(Node):
    def __init__(self):
        super().__init__('ik_planner')

        # ---- Clients ----
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /apply_planning_scene")

    def add_collision_object(self):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # box.dimensions = [item.length, item.width, item.height]
        box.dimensions = [3.0,3.0,3.0]
        box_pose = Pose()
        box_pose.position.x = 1.0
        box_pose.position.y = 1.0
        box_pose.position.z = 1.0
        box_pose.orientation.w = 1.0


        coll_obj = CollisionObject()
        coll_obj.header.frame_id = 'world'
        coll_obj.id = '67'
        coll_obj.primitives = [box]
        coll_obj.primitive_poses = [box_pose]
        coll_obj.operation = CollisionObject.ADD

        scene_msg = PlanningScene()
        scene_msg.world.collision_objects.append(coll_obj)
        scene_msg.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg

        self.get_logger().info('Calling service')
        future = self.scene_cli.call_async(req)
        future.add_done_callback(self.collision_response_callback)

    def collision_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Success')
            else:
                self.get_logger().info('Fail add')
        except Exception as e:
            self.get_logger().info(f'Fail: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    node.add_collision_object()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()