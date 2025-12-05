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
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from packing.main import packing_with_priors
from packing.config import PackingConfig


from box_bin_msgs.msg import BoxBin

# Assuming this import exists in your workspace
from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.box_pose_array_sub = self.create_subscription(BoxBin, '/box_bin', self.cube_callback, 1) 
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1) 

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

    def cube_callback(self, msg: BoxBin):
        # if self.cube_pose is not None:
        #     return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed.")
            return

        if not msg.box_poses:
            self.get_logger().info("message received but no box poses yet.")
            return

        box_ids = msg.box_ids
        box_poses = msg.box_poses
        bin_id = msg.bin_ids[0]
        bin_pose = msg.bin_poses[0]

        i = 0
        id = box_ids[0]
        
        # for i, id in enumerate(box_ids):
        
        # print(f"\n[Hello] Press any key to process Box ID {id}...", end='', flush=True)
        # input() 
        
        box_pose = box_poses[i]
        box = get_object_by_id(id)
        
        # self.cube_pose = box_pose # NOTE: not sure what this does... maybe to persist the current working cube pose?

        # 2. Determine Target Pose
        # final_pose_stamped = self.calculate_final_pose()
        # target_pose = final_pose_stamped.pose
        
        # NOTE: dummy target pose for now, should be replaced by actual pose from RL algorithm
        target_pose = Pose()
        target_pose.position.x = bin_pose.position.x
        target_pose.position.y = bin_pose.position.y
        target_pose.position.z = bin_pose.position.z + 0.15
        target_pose.orientation = bin_pose.orientation

        # 3. Plan and Execute
        # Pass the source_box_id to the planner
        success = self.plan_pick_and_place(box, box_pose, target_pose)
        
        if success:
            self.execute_jobs()
        else:
            self.get_logger().error(f"Planning failed for box {id}, clearing queue.")
            self.job_queue = []

    def plan_pick_and_place(self, box, source_pose, target_pose):
        """
        Plans the sequence with Selective ACM: 
        1. Allow Collision (Gripper <-> Box ONLY)
        2. Pre-grasp -> Grasp -> Internal Grip
        3. Attach Object (Update Planning Scene)
        4. Lift 
        5. Move to Target 
        6. Release -> Detach Object -> Retreat 
        7. Re-enable Collision (Gripper <-> Box)
        8. Return Home
        """
        
        # -----------------------------------------------------------
        # CONFIGURATION & MATH
        # -----------------------------------------------------------
        
        # 0) close the gripper
        self.job_queue.append('toggle_grip')
        
        cube_x = source_pose.position.x
        cube_y = source_pose.position.y
        cube_z = source_pose.position.z

        bin_x = target_pose.position.x
        bin_y = target_pose.position.y
        bin_z = target_pose.position.z

        # -----------------------------------------------------------
        # step 1: position and grasp

        # pose_grasp = self.ik_planner.compute_ik(self.joint_state, x_g, y_g, z_g, qx_src, qy_src, qz_src, qw_src)
        pose_grasp = self.ik_planner.compute_ik(self.joint_state, cube_x, cube_y, cube_z + 0.3)
        if not pose_grasp: return False
        self.job_queue.append(pose_grasp)

        # -----------------------------------------------------------
        # step 2: grab, attach, lift

        self.job_queue.append('toggle_grip')
        
        # -----------------------------------------------------------
        # step 3: move to place location

        # pose_place = self.ik_planner.compute_ik(self.joint_state, x_place, y_place, z_place, qx_src, qy_src, qz_src, qw_src)
        pose_place = self.ik_planner.compute_ik(self.joint_state, bin_x, bin_y, bin_z + 0.3)
        if not pose_place: return False
        self.job_queue.append(pose_place)

        # -----------------------------------------------------------
        # step 4: release and detach

        self.job_queue.append('toggle_grip')

        HOME_X, HOME_Y, HOME_Z = 0.3, 0.0, 0.5 
        # pose_home = self.ik_planner.compute_ik(self.joint_state, HOME_X, HOME_Y, HOME_Z, qx_src, qy_src, qz_src, qw_src)
        pose_home = self.ik_planner.compute_ik(self.joint_state, HOME_X, HOME_Y, HOME_Z)
        
        if pose_home:
            self.job_queue.append(pose_home)
        else:
            self.get_logger().warn("Could not plan to Home, finishing at retreat pos.")

        return True

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


        # HANDLE ATTACH
        elif isinstance(next_job, tuple) and next_job[0] == 'attach_box':
            _, box_id = next_job
            self.get_logger().info(f"Attaching object: {box_id}")
            # Ensure you have a method to attach the box in your class
            if hasattr(self, 'attach_box'):
                self.attach_box(box_id)
            else:
                self.get_logger().warn("attach_box method missing!")
            self.execute_jobs()

        # HANDLE DETACH
        elif isinstance(next_job, tuple) and next_job[0] == 'detach_box':
            _, box_id = next_job
            self.get_logger().info(f"Detaching object: {box_id}")
            # Ensure you have a method to detach the box in your class
            if hasattr(self, 'detach_box'):
                self.detach_box(box_id)
            else:
                self.get_logger().warn("detach_box method missing!")
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