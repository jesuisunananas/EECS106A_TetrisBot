# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseArray
from moveit_msgs.msg import RobotTrajectory, PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from ros2_aruco_interfaces.msg import ArucoMarkers
from shared_things import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

# Assuming this import exists in your workspace
from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.box_pose_array_sub = self.create_subscription(PoseArray, '/box_aruco_poses', self.cube_callback, 1) 
        
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

        self.job_queue = [] 

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def update_acm(self, allow=True):
        """
        Updates the Allowed Collision Matrix to allow/disallow collisions 
        between the gripper and everything else.
        """
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        
        acm = AllowedCollisionMatrix()
        
        gripper_links = ['robotiq_85_base_link', 'robotiq_85_left_inner_knuckle_link', 
                         'robotiq_85_left_finger_tip_link', 'robotiq_85_left_knuckle_link', 
                         'robotiq_85_right_inner_knuckle_link', 'robotiq_85_right_finger_tip_link', 
                         'robotiq_85_right_knuckle_link', 'wrist_3_link']
        
        acm.entry_names = gripper_links
        
        for _ in gripper_links:
            entry = AllowedCollisionEntry()
            entry.enabled = [not allow] * len(gripper_links) 
            acm.entry_values.append(entry)

        scene_msg.allowed_collision_matrix = acm
        self.scene_pub.publish(scene_msg)
        self.get_logger().info(f"ACM Updated: Gripper Collisions Allowed = {allow}")

    def cube_callback(self, msg: PoseArray):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        if not msg.poses:
            self.get_logger().info("PoseArray received but it is empty.")
            return

        target_pose = msg.poses[0]
        self.cube_pose = target_pose 

        # -----------------------------------------------------------
        # INTERNAL SIDE GRASP STRATEGY
        # -----------------------------------------------------------

        # 0) Ensure Gripper is Closed
        self.job_queue.append('toggle_grip')

        CUBE_HEIGHT = 0.08 # NOTE: magic number for debugging

        # Extract cube position
        x_initial = float(target_pose.position.x)
        y_initial = float(target_pose.position.y)
        z_initial = float(target_pose.position.z)

        # --- UPDATED ORIENTATION LOGIC START ---
        
        # 1. Get the FULL orientation of the cube (Pitch, Roll, and Yaw)
        r_cube = R.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ])

        # 2. Define the rotation required to go from "Marker Frame" to "Side Grasp Frame"
        #    Assuming Marker Z is UP. We want Gripper Z (Approach) to be horizontal relative to marker.
        #    Rotate 90 degrees around Y axis.
        r_side_offset = R.from_euler('y', 90, degrees=True)

        # 3. Combine them. By multiplying r_cube * r_side_offset, 
        #    we apply the side grasp rotation locally to the cube's specific orientation.
        #    This preserves the cube's tilt.
        r_target = r_cube * r_side_offset
        
        rx, ry, rz = r_target.as_rotvec()

        vec_pre_grasp_local = [-0.10, 0.0, -CUBE_HEIGHT/2.0]
        
        # Transform this local vector into World frame using the CUBE'S full rotation
        offset_pre = r_cube.apply(vec_pre_grasp_local)
        
        x_pre_grasp = x_initial + offset_pre[0]
        y_pre_grasp = y_initial + offset_pre[1]
        z_pre_grasp = z_initial + offset_pre[2] # Use z_initial + offset
        
        # NOTE: hack: allow collision
        self.update_acm(allow=True)

        target_pre_grasp_pose = self.ik_planner.compute_ik(self.joint_state, x_pre_grasp, y_pre_grasp, z_pre_grasp, rx, ry, rz)
        if target_pre_grasp_pose is None:
            self.get_logger().error(f"IK failed for pre grasp")
            return False
        self.job_queue.append(target_pre_grasp_pose)

        # 2) Grasp Position: EXTEND INTO OBJECT BY 20mm
        # We move from the pre-grasp closer to the center. 
        # Let's define the grasp point relative to the marker center again.
        # Ideally, we want to be "inside" the cube.
        # Vector: +0.02m inside relative to side face? 
        # If side face is at X = -0.04 (half width), and we want 20mm deep, we might want X = -0.02?
        # Let's assume we simply drive 12cm forward from the pre-grasp point along the approach vector.
        
        # Alternatively, define strictly relative to Marker Center:
        # If Marker is center, side face is at -0.04. We want to be at -0.04 + 0.02 = -0.02
        vec_grasp_local = [-0.02, 0.0, -CUBE_HEIGHT/2.0]
        
        offset_grasp = r_cube.apply(vec_grasp_local)
        
        x_grasp = x_initial + offset_grasp[0]
        y_grasp = y_initial + offset_grasp[1]
        z_grasp = z_initial + offset_grasp[2]

        target_grasp_pose = self.ik_planner.compute_ik(self.joint_state, x_grasp, y_grasp, z_grasp, rx, ry, rz)
        if not target_grasp_pose:
            self.get_logger().error(f"IK failed for grasp pose")
            return False
        self.job_queue.append(target_grasp_pose)

        # 3) Open the gripper (Internal/Expansion Grasp)
        self.job_queue.append('toggle_grip')
        
        # 4) Retreat / Lift
        # We lift relative to the WORLD Z usually, or relative to Cube Z.
        # Let's lift straight UP in World Z for safety.
        target_lift_pose = self.ik_planner.compute_ik(self.joint_state, x_grasp, y_grasp, z_grasp + 0.2, rx, ry, rz)
        if not target_lift_pose:
            self.get_logger().error(f"IK failed for lift pose")
            return False
        self.job_queue.append(target_lift_pose)

        # 5) Move to release Position
        
        #TODO: uhhhhh
        final_pose = self.calculate_final_pose()
        #TODO: uhhhhh
        
        place_x = final_pose.pose.position.x
        place_y = final_pose.pose.position.y
        place_z = final_pose.pose.position.z + 0.03
        target_move_pose = self.ik_planner.compute_ik(self.joint_state, place_x, place_y, place_z, rx, ry, rz)
        if not target_move_pose:
            self.get_logger().error(f"IK failed for move pose")
            return False
        self.job_queue.append(target_move_pose)

        # 6) Close the gripper (Contract to Release)
        self.job_queue.append('toggle_grip')

        # 7) Restore Collision Checks
        self.job_queue.append('disable_acm') 

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
            self.execute_jobs() 
            
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs() 

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  
          
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
            self.execute_jobs() 
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

    def calculate_final_pose(self, box_info: tuple, bin_tf):
        id, name, fragility, z_base, z_top, x, y = box_info #TODO ask arjun to include an ID
        
        box = get_object_by_id(id)
        pose = PoseStamped()
        
        pose.header.frame_id = bin_tf.child_frame_id
        pose.header.time = rclpy.time.Time()
        
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        pose.position.x = x + (box.width / 2.0)
        pose.position.y = y + (box.length / 2.0)
        pose.position.z = z_base + (box.height / 2.0)
        
        return do_transform_pose(pose, bin_tf)

def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()