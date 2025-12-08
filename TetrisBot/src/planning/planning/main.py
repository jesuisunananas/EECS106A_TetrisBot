# ROS Libraries
from std_srvs.srv import Trigger, Empty
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
# from geometry_msgs.msg import PointStamped, PoseArray
from moveit_msgs.msg import RobotTrajectory, PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
# from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from ros2_aruco_interfaces.msg import ArucoMarkers
from shared_things import *
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
# from packing.main import packing_with_priors
# from packing.config import PackingConfig
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject


from box_bin_msgs.msg import BoxBin

# Assuming this import exists in your workspace
from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.box_pose_array_sub = self.create_subscription(BoxBin, '/box_bin', self.objects_callback, 1) 
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1) 

        # Publisher for updating the Planning Scene (ACM)
        # self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        # Create a server for calling placing service
        self._client = self.create_service(Empty, '/run_packing', self._placing_service)

        # self.cube_pose = None
        self.current_plan = None
        self.joint_state = None
        self.current_objects = None
        self.is_busy = False

        self.ik_planner = IKPlanner()

        self.job_queue = [] 

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg
    
    def objects_callback(self, msg: BoxBin):
        if msg.box_ids:
            self.current_objects = msg

    def _placing_service(self, request, response):
        while self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed.")

        while not self.current_objects:
            self.get_logger().info("No poses yet.")
        
        msg = self.current_objects
        box_ids = msg.box_ids
        box_poses = msg.box_poses
        # bin_id = msg.bin_ids[0]
        bin_pose = msg.bin_poses[0]

        print(box_ids)

        # ---------------------------------------------------------
        # TODO this is just for initial test, change for more boxes
        box = get_object_by_id(box_ids[0])
        initial_pose = box_poses[0]
        # final_box = get_object_by_id(box_ids[1])
        final_pose = bin_pose
        self.get_logger().info(f'box pose {initial_pose.position.z}')
        self.get_logger().info(f'final callback pose {final_pose.position.z}')

        # initial_pose.position.y += GRIPPER_OFFSET_Y
        # initial_pose.position.z += (GRIPPER_OFFSET_Z + (box.height / 2))
        # final_pose.position.y += GRIPPER_OFFSET_Y
        # final_pose.position.z += (GRIPPER_OFFSET_Z + (final_box.height / 2))
        # final_pose.position.z += GRIPPER_OFFSET_Z 
        
        success = self.test_plan_pick_and_place(box, initial_pose, final_pose)

        if success:
            self.get_logger().info(f"Planning successful for box {id}, running queue.")
            self.execute_jobs()
        else:
            self.get_logger().error(f"Planning failed for box {id}, clearing queue.")
            self.job_queue = []
        # ---------------------------------------------------------
        
        # final_poses, final_ids = ... #TODO: figure out how to get final poses info
        # for i, _ in enumerate(final_poses):
        #     try: 
        #         box_id = box_ids.index(final_ids[i])
        #     except ValueError:
        #         self.get_logger().error(f"Unable to find info for box {final_ids[i]}, clearing queue.")
        #         self.job_queue = []

        #     box = get_object_by_id(box_id)
        #     initial_pose = box_poses[box_id]
        #     final_pose = final_poses[i]
        #     success = self.test_plan_pick_and_place(box, initial_pose, final_pose)

        #     if success:
        #         self.get_logger().info(f"Planning successful for box {id}, running queue.")
        #         self.is_busy = True
        #         self.execute_jobs()
        #     else:
        #         self.get_logger().error(f"Planning failed for box {id}, clearing queue.")
        #         self.job_queue = []

        #     while self.is_busy:
        #         self.get_logger().info(f'Waiting for box {id} to finish')
        
        return response

    def test_plan_pick_and_place(self, box, source_pose, target_pose):
        
        # clear out job queue:
        self.job_queue = []
        
        # 0) close the gripper
        self.job_queue.append('toggle_grip')
        
        # 1) Pregrasp at 20cm above the cube surface
        x_pre = source_pose.position.x
        y_pre = source_pose.position.y
        z_pre = source_pose.position.z + GRIPPER_OFFSET_Z + (box.width/2) + 0.2 - 0.01
        ik_result = self.ik_planner.compute_ik(self.joint_state, x_pre, y_pre, z_pre)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        self.get_logger().info(f'hovering 20cm above the cube surface at z={z_pre}')
        self.get_logger().info(f'move down by 20cm to z={z_pre - 0.2}')

        # 2) Grasp position
        ik_result = self.ik_planner.compute_ik(ik_result, x_pre, y_pre, z_pre - 0.2) 
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 3) Open gripper
        self.job_queue.append('toggle_grip')

        # 4) Move up
        ik_result = self.ik_planner.compute_ik(ik_result, x_pre, y_pre, z_pre)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 5) Move to the x and y position of final pose
        x_final = target_pose.position.x
        y_final = target_pose.position.y
        z_final = target_pose.position.z + GRIPPER_OFFSET_Z + 0.2
        ik_result = self.ik_planner.compute_ik(ik_result, x_final, y_final, z_final)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 6) Lower to final pose z
        ik_result = self.ik_planner.compute_ik(ik_result, x_final, y_final, z_final - 0.2 + (box.width) + 0.02)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 7) Close gripper to release
        self.job_queue.append('toggle_grip')
        self.get_logger().info(f'final pose at z={z_final}')

        # 8) Move back to above final pose
        ik_result = self.ik_planner.compute_ik(ik_result, x_final, y_final, z_final)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        return True


    def plan_pick_and_place(self, box, source_pose, target_pose):
        """
        Plans the sequence with Selective ACM: 
        1. Allow Collision (Gripper <-> Box ONLY) <--NO
        2. Pre-grasp -> Grasp -> Internal Grip
        3. Attach Object (Update Planning Scene) <--NO
        4. Lift 
        5. Move to Target 
        6. Release -> Detach Object -> Retreat 
        7. Re-enable Collision (Gripper <-> Box) <--NO
        8. Return Home
        """
        box_id = box.id

        # --- Orientation Logic ---
        r_source = R.from_quat([
            source_pose.orientation.x, 
            source_pose.orientation.y,
            source_pose.orientation.z, 
            source_pose.orientation.w
        ])
        
        
        r_dest = R.from_quat([
            target_pose.orientation.x, 
            target_pose.orientation.y,
            target_pose.orientation.z, 
            target_pose.orientation.w
        ])

        r_side_offset = [0, 0, 1] # z-axis of box-frame

        r_source_z_axis = r_source.apply(r_side_offset) # get in base-link frame
        r_dest_z_axis = r_dest.apply(r_side_offset)

        y_axis_base = [0, 1, 0] #should be pointing out towards from base_link
        r_source_ee, _, _ = R.align_vectors(r_source_z_axis, y_axis_base) # get rotation from ee pointing out towards 
                                                                        # to z-axis of box frame
        r_dest_ee, _, _ = R.align_vectors(r_dest_z_axis, y_axis_base)

        r_ee = R.from_quat([0, 1, 0, 0])

        r_source_ee = r_source_ee.as_matrix() @ r_ee.as_matrix() 
        r_dest_ee = r_dest_ee.as_matrix() @ r_ee.as_matrix()

        r_source_ee = R.from_matrix(r_source_ee)
        r_dest_ee = R.from_matrix(r_dest_ee)

        qx_src, qy_src, qz_src, qw_src = r_source_ee.as_quat() # Convert to quaternion for IK (IMPORTANT!)
        qx_dst, qy_dst, qz_dst, qw_dst = r_dest_ee.as_quat()
        
        
        # CALCULATE PRE-GRASP AND GRASP POSITIONS:
        # Position the end-effector with 5cm standoff from the box face along the x-axis
        # TODO: figure out if this would center in box (x & y wise)
        # pre_grasp_local = [(box.width / 2.0), 0.0, (0.05 - box.height / 2.0)]
        # Push the end affector 2cm into the box face along the x-axis
        # grasp_local = [(box.width / 2.0), 0.0, (- box.height / 2.0  - 0.02)]


        # -----------------------------------------------------------
        # CONFIGURATION & MATH
        # -----------------------------------------------------------
        
        # 0) close the gripper
        self.job_queue.append('toggle_grip')

        # -----------------------------------------------------------
        # step 1: position and grasp
        
        # Calculate Source Pre-Grasp
        # pre_grasp_base_link = r_source.apply(pre_grasp_local)
        x_pre = source_pose.position.x #+ pre_grasp_base_link[0]
        y_pre = source_pose.position.y #+ pre_grasp_base_link[1]
        z_pre = source_pose.position.z + GRIPPER_OFFSET_Z + (box.width / 2) + 0.2 #+ pre_grasp_base_link[2]

        ik_result = self.ik_planner.compute_ik(self.joint_state, x_pre, y_pre, z_pre, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for pre grasp")
            return False
        self.job_queue.append(ik_result)

        # Calculate Source Grasp (Entering the object)
        grasp_base_link = r_source.apply(grasp_local)
        x_g = source_pose.position.x + grasp_base_link[0]
        y_g = source_pose.position.y + grasp_base_link[1]
        z_g = source_pose.position.z + grasp_base_link[2]

        ik_result = self.ik_planner.compute_ik(ik_result, x_g, y_g, z_g, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for post grasp")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # step 2: grab, lift

        # Grab
        self.job_queue.append('toggle_grip')

        # Lift
        ik_result = self.ik_planner.compute_ik(ik_result, x_g, y_g, z_g + 0.2, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for lift")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # step 3: move to place location

        # based on the target pose position, add an offset to accommodate for the end effector being slightly inside the cube
        grasp_offset = r_dest.apply(grasp_local)
        x_place = target_pose.position.x + grasp_offset[0]
        y_place = target_pose.position.y + grasp_offset[1]
        z_place = target_pose.position.z + 0.03 + grasp_offset[2] # 3cm to the z so the end effector is 
                                                                    # slightly above the placement surface

        # Position above place location before lowering to place location
        ik_result = self.ik_planner.compute_ik(ik_result, x_place, y_place, z_place + 0.15, qx_dst, qy_dst, qz_dst, qw_dst)
        if not ik_result: 
            self.get_logger().error("IK failed for above place")
            return False
        self.job_queue.append(ik_result)
        
        # Move to place location
        ik_result = self.ik_planner.compute_ik(ik_result, x_place, y_place, z_place, qx_dst, qy_dst, qz_dst, qw_dst)
        if not ik_result: 
            self.get_logger().error("IK failed for place")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # step 4: release

        self.job_queue.append('toggle_grip')

        # -----------------------------------------------------------
        # step 5: back out of box

        offset_pre_dst = r_dest.apply(pre_grasp_local)
        x_retreat = target_pose.position.x + offset_pre_dst[0]
        y_retreat = target_pose.position.y + offset_pre_dst[1]
        z_retreat = target_pose.position.z + offset_pre_dst[2]

        # Move up to above place location
        ik_result = self.ik_planner.compute_ik(ik_result, x_retreat, y_retreat, z_retreat + 0.15, qx_dst, qy_dst, qz_dst, qw_dst)
        if not ik_result: 
            self.get_logger().error("IK failed for retreat")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # step 6: re-enable collision checking and home
        
        # HOME_X, HOME_Y, HOME_Z = 0.3, 0.0, 0.5 
        # pose_home = self.ik_planner.compute_ik(self.joint_state, HOME_X, HOME_Y, HOME_Z, qx_dst, qy_dst, qz_dst, qw_dst)
        
        # if pose_home:
        #     self.job_queue.append(pose_home)
        # else:
        #     self.get_logger().warn("Could not plan to Home, finishing at retreat pos.")

        # Moves to home position (tuck)
        # self.job_queue.append('tuck')

        #NOTE: maybe don't need to go back to home position? as long as arms out of way i guess
        # ik_result = self.ik_planner.compute_ik(ik_result, x_retreat, y_retreat, z_retreat, qx_dst, qy_dst, qz_dst, qw_dst)
        # if not ik_result: 
        #     self.get_logger().error("IK failed for retreat")
        #     return False
        # self.job_queue.append(ik_result)

        return True

    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            # rclpy.shutdown()
            self.is_busy = False
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

        # elif next_job == 'tuck':
        #     self.get_logger().info('Calling tuck function')
        #     self._tuck()
            
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
    
    # def _tuck(self):
    #     # NOTE: figure out how to do go to home/tuck
    #     self.execute_jobs()
          
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
        # For changing from bin-frame coor to base-link frame
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
        pose.position.y = y - (box.length / 2.0)
        pose.position.z = z_base + (box.height / 2.0)
        
        return do_transform_pose(pose, bin_tf)

def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()