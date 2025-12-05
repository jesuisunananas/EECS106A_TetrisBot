import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


# Example usage:
# -------------------------------------------------
# current joint state (replace with your robot's)
# current_state = JointState()
# current_state.name = [
#     'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
# ]
# current_state.position = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]

# Compute IK for target point
# ik_solution = node.compute_ik(current_state, 0.4, 0.1, 0.3)

# if ik_solution:
#     # Plan motion to the found joint configuration
#     trajectory = node.plan_to_joints(ik_solution)
#     if trajectory:
#         node.get_logger().info('Trajectory ready to execute.')


class IKPlanner(Node):
    def __init__(self):
        super().__init__('ik_planner')

        # ---- Clients ----
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        for srv, name in [(self.ik_client, 'compute_ik'),
                          (self.plan_client, 'plan_kinematic_path')]:
            while not srv.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for /{name} service...')

    # -----------------------------------------------------------
    # TODO: Compute IK for a given (x, y, z) + quat and current robot joint state
    # -----------------------------------------------------------
    def compute_ik(self, current_joint_state, x, y, z,
                   qx=0.0, qy=1.0, qz=0.0, qw=0.0): # Think about why the default quaternion is like this. Why is qy=1?
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        ik_req = GetPositionIK.Request()
        # ik_req.ik_request.ik_link_name = 'tool0'     # NOTE TO KEVIN might have to change if your EE link differs
        ik_req.ik_request.pose_stamped = pose
        ik_req.ik_request.robot_state.joint_state = current_joint_state
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.timeout = Duration(sec=2)
        ik_req.ik_request.group_name = 'ur_manipulator'
        

        future = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('IK service failed.')
            return None

        result = future.result()
        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(f'IK failed, code: {result.error_code.val}')
            return None

        self.get_logger().info('IK solution found.')
        return result.solution.joint_state

    # -----------------------------------------------------------
    # Plan motion given a desired joint configuration
    # -----------------------------------------------------------
    def plan_to_joints(self, target_joint_state):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'ur_manipulator'
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.planner_id = "RRTConnectkConfigDefault"

        goal_constraints = Constraints()
        for name, pos in zip(target_joint_state.name, target_joint_state.position):
            goal_constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=pos,
                    tolerance_above=0.01,
                    tolerance_below=0.01,
                    weight=1.0
                )
            )

        req.motion_plan_request.goal_constraints.append(goal_constraints)
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Planning service failed.')
            return None

        result = future.result()
        if result.motion_plan_response.error_code.val != 1:
            self.get_logger().error('Planning failed.')
            return None

        self.get_logger().info('Motion plan computed successfully.')
        return result.motion_plan_response.trajectory


def main(args=None):
    rclpy.init(args=args)
    node = IKPlanner()

    # ---------- Test setup ----------
    current_state = JointState()
    current_state.name = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]

    # 4.722274303436279
    # -1.8504554233946742
    # -1.4257320165634155
    # -1.4052301210216065
    # 1.5935229063034058
    # -3.14103871980776

    current_state.position = [4.722, -1.850, -1.425, -1.405, 1.593, -3.141]

    # ---------- Run IK ----------
    node.get_logger().info("Testing IK computation...")
    ik_result = node.compute_ik(current_state, 0.125, 0.611, 0.423)

    # ---------- Check correctness ----------
    if ik_result is None:
        node.get_logger().error("IK computation returned None.")
        sys.exit(1)

    if not hasattr(ik_result, "name") or not hasattr(ik_result, "position"):
        node.get_logger().error("IK result missing required fields (name, position).")
        sys.exit(1)

    if len(ik_result.name) != len(ik_result.position):
        node.get_logger().error("IK joint names and positions length mismatch.")
        sys.exit(1)

    if len(ik_result.name) < 6:
        node.get_logger().error("IK returned fewer than 6 joints — likely incorrect.")
        sys.exit(1)

    node.get_logger().info("IK check passed.")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
