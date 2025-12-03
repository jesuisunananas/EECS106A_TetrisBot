# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseArray # Added PoseArray
from moveit_msgs.msg import RobotTrajectory, PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry # Added ACM msgs
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        # CHANGED: Subscription is now for PoseArray instead of PointStamped
        # Make sure '/aruco_poses' matches the topic publishing your array of markers
        self.pose_array_sub = self.create_subscription(PoseArray, '/aruco_poses', self.cube_callback, 1) 
        
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        # Publisher for updating the Planning Scene (ACM)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.current_plan = None
        self.joint_state = None

        self.ik_planner = IKPlanner()

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def update_acm(self, allow=True):
        """
        Option B: Updates the Allowed Collision Matrix to allow/disallow collisions 
        between the gripper and everything else.
        """
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        
        acm = AllowedCollisionMatrix()
        
        # Define gripper links (Adjust these names to match your specific URDF/Gripper)
        # Common names for Robotiq 2F-85 or similar:
        gripper_links = ['robotiq_85_base_link', 'robotiq_85_left_inner_knuckle_link', 
                         'robotiq_85_left_finger_tip_link', 'robotiq_85_left_knuckle_link', 
                         'robotiq_85_right_inner_knuckle_link', 'robotiq_85_right_finger_tip_link', 
                         'robotiq_85_right_knuckle_link', 'wrist_3_link']
        
        acm.entry_names = gripper_links
        
        for _ in gripper_links:
            entry = AllowedCollisionEntry()
            entry.enabled = [not allow] * len(gripper_links) # This enables/disables self-collision
            # To strictly allow collision with the WORLD/OBJECTS, we usually need the object names.
            # Since we don't have object IDs here, we might need a broader wildcard or assume 
            # the planner respects the 'default' entry if we could set it.
            # However, a robust way without object IDs is tricky. 
            # Ideally, we would set entry.enabled = [True] for ALL objects in the scene.
            # For now, we will assume this toggle logic is sufficient or handled by specific planner settings.
            # NOTE: For a simple hack, MoveIt often requires explicit object names.
            # If this doesn't work, you might need to know the name of the 'cube' collision object.
            acm.entry_values.append(entry)

        # If we knew the collision object name (e.g. 'table_cube'), we would do:
        # acm.entry_names = ['table_cube']
        # entry.enabled = [True] # Allow collision
        
        scene_msg.allowed_collision_matrix = acm
        self.scene_pub.publish(scene_msg)
        self.get_logger().info(f"ACM Updated: Gripper Collisions Allowed = {allow}")

    def cube_callback(self, msg: PoseArray):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        # CHANGED: Check if the array is empty
        if not msg.poses:
            self.get_logger().info("PoseArray received but it is empty.")
            return

        # CHANGED: Extract the first pose from the array
        target_pose = msg.poses[0]
        self.cube_pose = target_pose 

        # -----------------------------------------------------------
        # INTERNAL SIDE GRASP STRATEGY (OPTION B)
        # 1. Close Gripper
        # 2. Allow Collision (Hack)
        # 3. Orient Horizontally (Side Approach)
        # 4. Enter Cube (20mm deep)
        # 5. Open Gripper
        # -----------------------------------------------------------

        # 0) Ensure Gripper is Closed
        self.job_queue.append('toggle_grip')

        # Define Cube Geometry
        CUBE_HEIGHT = 0.08 # 8cm cube. Adjust as needed.

        # Extract cube position (Marker Position usually on TOP surface)
        x_initial = float(target_pose.position.x)
        y_initial = float(target_pose.position.y)
        z_initial = float(target_pose.position.z)

        # ORIENTATION LOGIC
        r_cube = R.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])
        yaw = r_cube.as_euler('xyz')[2] 
        r_yaw = R.from_euler('z', yaw)

        # Side Grasp Orientation (Z points horizontal)
        # Rotates standard Z-axis (down) to point along Local +X axis
        r_side = R.from_euler('y', 90, degrees=True)
        r_target = r_yaw * r_side 
        rx, ry, rz = r_target.as_rotvec()

        # --- OPTION B: ALLOW COLLISION ---
        # We must allow collision NOW so the IK planner doesn't reject the 'inside' point
        self.update_acm(allow=True)

        # POSITION LOGIC
        # We need to grasp the CENTER of the side face.
        # Since z_initial is the top, we move down by height/2.
        z_center = z_initial - (CUBE_HEIGHT / 2.0)
        
        # 1) Pre-Grasp: 10cm away from the side
        offset_pre = r_yaw.apply([-0.10, 0.0, 0.0])
        x_pre_grasp = x_initial + offset_pre[0]
        y_pre_grasp = y_initial + offset_pre[1]
        z_pre_grasp = z_center # Approach at center height
        
        target_pre_grasp_pose = self.ik_planner.compute_ik(self.joint_state, x_pre_grasp, y_pre_grasp, z_pre_grasp, rx, ry, rz)
        if target_pre_grasp_pose is None:
            self.get_logger().error(f"IK failed for pre grasp")
            return False
        self.job_queue.append(target_pre_grasp_pose)

        # 2) Grasp Position: EXTEND INTO OBJECT BY 20mm
        # Previous: center (0.0). Now: +0.02m along local X.
        # Direction: From -X to +X.
        offset_grasp = r_yaw.apply([0.02, 0.0, 0.0]) # 20mm positive offset
        
        x_grasp = x_initial + offset_grasp[0]
        y_grasp = y_initial + offset_grasp[1]
        z_grasp = z_center # Grasp at center height

        target_grasp_pose = self.ik_planner.compute_ik(self.joint_state, x_grasp, y_grasp, z_grasp, rx, ry, rz)
        if not target_grasp_pose:
            self.get_logger().error(f"IK failed for grasp pose")
            return False
        self.job_queue.append(target_grasp_pose)

        # 3) Open the gripper (Internal/Expansion Grasp)
        self.job_queue.append('toggle_grip')
        
        # 4) Retreat / Lift
        target_lift_pose = self.ik_planner.compute_ik(self.joint_state, x_grasp, y_grasp, z_grasp + 0.2, rx, ry, rz)
        if not target_lift_pose:
            self.get_logger().error(f"IK failed for lift pose")
            return False
        self.job_queue.append(target_lift_pose)

        # 5) Move to release Position
        place_x = x_initial + 0.4
        place_y = y_initial
        place_z = z_grasp + 0.1
        target_move_pose = self.ik_planner.compute_ik(self.joint_state, place_x, place_y, place_z, rx, ry, rz)
        if not target_move_pose:
            self.get_logger().error(f"IK failed for move pose")
            return False
        self.job_queue.append(target_move_pose)

        # 6) Close the gripper (Contract to Release)
        self.job_queue.append('toggle_grip')

        # 7) Restore Collision Checks
        self.job_queue.append('disable_acm') # Add custom job to queue

        self.execute_jobs()

    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):
            traj = self.ik_planner.plan_to_joints(next_job)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return
            self.get_logger().info("Planned to position")
            self._execute_joint_trajectory(traj.joint_trajectory)
            
        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
            
        elif next_job == 'disable_acm':
            self.update_acm(allow=False)
            self.execute_jobs() # Immediately process next
            
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job

            
    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()