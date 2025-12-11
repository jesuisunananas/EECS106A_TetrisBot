# ROS Libraries
from std_srvs.srv import Trigger, Empty
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
# from tf2_ros import Buffer, TransformListener
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np

from ros2_aruco_interfaces.msg import ArucoMarkers
from shared_things import *
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from shared_things.packing.main import packing_with_priors
from shared_things.packing.config import PackingConfig
import time

from box_bin_msgs.msg import BoxBin

# Assuming this import exists in your workspace
from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.box_pose_array_sub = self.create_subscription(BoxBin, '/box_bin', self.objects_callback, 1) 
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1) 
        
        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        # Create a server for calling placing service
        self._client = self.create_service(Empty, '/run_packing', self._placing_service)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joint_state = None
        self.current_objects = None
        self.is_busy = False

        self.ik_planner = IKPlanner()

        self.job_queue = [] 

    def joint_state_callback(self, msg: JointState):
        if not self.is_busy:
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
        bin_id = msg.bin_ids[0]
        bin_pose = msg.bin_poses[0]

        print(box_ids)

        # ---------------------------------------------------------
        # NOTE: Demo 1: this is just for initial test, change for more boxes
        # box = get_object_by_id(box_ids[0])
        # initial_pose = box_poses[0]
        # # final_box = get_object_by_id(box_ids[1])
        # final_pose = bin_pose
        # self.get_logger().info(f'box pose {initial_pose.position.z}')
        # self.get_logger().info(f'final callback pose {final_pose.position.z}')

        # # initial_pose.position.y += GRIPPER_OFFSET_Y
        # # initial_pose.position.z += (GRIPPER_OFFSET_Z + (box.height_m / 2))
        # # final_pose.position.y += GRIPPER_OFFSET_Y
        # # final_pose.position.z += (GRIPPER_OFFSET_Z + (final_box.height_m / 2))
        # # final_pose.position.z += GRIPPER_OFFSET_Z 
        
        # # success = self.test_plan_pick_and_place(box, initial_pose, final_pose)
        # success = self.plan_pick_and_place(box, initial_pose, final_pose)

        # if success:
        #     self.get_logger().info(f"Planning successful for box {id}, running queue.")
        #     self.execute_jobs()
        # else:
        #     self.get_logger().error(f"Planning failed for box {id}, clearing queue.")
        #     self.job_queue = []
        # ---------------------------------------------------------
        
        # ---------------------------------------------------------
        # NOTE: Demo 2: Stack multiple cubes, pausing each time for user input
        # self.is_busy = True
        # for id in box_ids:
        #     box = get_object_by_id(id)
        #     # input(f"Press Enter to move box {box.name}, id: {id}:")
        #     self.get_logger().info(f"Planning box {box.name}, {id}...")
        #     initial_pose = box_poses[box_ids.index(id)]
        #     final_pose = bin_pose 
        #     # TODO: ^^^The location of the bin AR_marker should actually 
        #     # correspond to the top left corner of the bin pose, so placed 
        #     # blocks doesn't cover the bin marker.
        #     # TODO: fix in ar_tag_identify.py
            
        #     self.get_logger().info(f'box pose {initial_pose.position.z}')
        #     self.get_logger().info(f'final callback pose {final_pose.position.z}')

        #     # success = self.plan_pick_and_place(box, initial_pose, final_pose)
        #     success = self.test_plan_pick_and_place(box, initial_pose, final_pose)

        #     if success:
        #         self.get_logger().info(f"Planning successful for box {id}! Adding to queue.")
        #         bin_pose.position.z += box.width_m
        #     else:
        #         self.get_logger().error(f"Planning failed for box {id}, clearing queue.")
        #         self.job_queue = []
        #         self.is_busy = False
        #         return response

        # if self.job_queue:
        #     self.execute_jobs()
        # return response
        # ---------------------------------------------------------
        # # NOTE: Demo 3: Stacking cubes based on packing_with_priors and prioirity order
        box_list = [get_object_by_id(id) for id in box_ids]
        box_info = packing_with_priors(box_list=box_list, vis=False)
        
        self.is_busy = True
        for info in box_info:
            box_id = info[0]
            box_idx = box_ids.index(box_id)
            box = get_object_by_id(box_id)
            initial_pose = box_poses[box_idx]
            
            # NOTE: the bin orientation shouldn't actually matter tbh. 
            # Placing based on the top left corner should still work
            # transform_timeout = rclpy.duration.Duration(seconds=0.1)
            # bin_tf = self.tf_buffer.lookup_transform("base_link", 
            #                                          f"ar_marker_{bin_id}", 
            #                                          rclpy.time.Time(), 
            #                                          timeout=transform_timeout
            #                                          )
            final_pose = self.calculate_final_pose(info, bin_pose, get_object_by_id(bin_id))

            self.get_logger().info(f"Dim for box {box_id}: l:{box.length_m}, w:{box.width_m}, h:{box.height_m}")

            success = self.plan_pick_and_place(box, initial_pose, final_pose)
            if success:
                self.get_logger().info(f"Planning successful for box {box_id}! Adding to queue.")
            else:
                self.get_logger().error(f"Planning failed for box {box_id}, clearing queue.")
                self.job_queue = []
                self.is_busy = False
                return response
        # ---------------------------------------------------------
        if self.job_queue:
            self.execute_jobs()
        return response

    def test_plan_pick_and_place(self, box, source_pose, target_pose):
        # clear out job queue:
        self.job_queue = []
        
        # 0) close the gripper
        self.job_queue.append('toggle_grip')
        
        # 1) Pregrasp at 20cm above the cube surface
        x_pre = source_pose.position.x
        y_pre = source_pose.position.y
        z_pre = source_pose.position.z + GRIPPER_OFFSET_Z + (box.width_m/2) + 0.2 - 0.01
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
        ik_result = self.ik_planner.compute_ik(ik_result, x_final, y_final, z_final - 0.2 + (box.width_m) + 0.02)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 7) Close gripper to release
        self.job_queue.append('toggle_grip')
        self.get_logger().info(f'final pose at z={z_final}')

        # 8) Move back to above final pose
        ik_result = self.ik_planner.compute_ik(ik_result, x_final, y_final, z_final)
        if not ik_result: return False
        self.job_queue.append(ik_result)

        # 7) Open gripper to reset
        self.job_queue.append('toggle_grip')

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

        # ===================== Orientation Logic ===================
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

        y_axis_base = np.array([[0, 1, 0]]) #should be pointing out towards from base_link
        r_source_ee, _ = R.align_vectors(np.atleast_2d(r_source_z_axis), y_axis_base) # get rotation from ee pointing out towards 
                                                                                      # to z-axis of box frame
        r_dest_ee, _ = R.align_vectors(np.atleast_2d(r_dest_z_axis), y_axis_base)

        r_ee = R.from_quat([0, 1, 0, 0])

        r_source_ee = r_source_ee.as_matrix() @ r_ee.as_matrix() 
        r_dest_ee = r_dest_ee.as_matrix() @ r_ee.as_matrix()

        r_source_ee = R.from_matrix(r_source_ee)
        r_dest_ee = R.from_matrix(r_dest_ee)

        qx_src, qy_src, qz_src, qw_src = r_source_ee.as_quat() # Convert to quaternion for IK (IMPORTANT!)
        qx_dst, qy_dst, qz_dst, qw_dst = r_dest_ee.as_quat()
        # ===================== Orientation Logic ===================

        # -----------------------------------------------------------
        # STEP 0: close the gripper
        self.job_queue.append('toggle_grip')

        # -----------------------------------------------------------
        # STEP 1: position and grasp
        
        # Calculate Source Pre-Grasp
        # Pre-grasp EE position 20cm above cube top surface:
        x_pre = source_pose.position.x 
        y_pre = source_pose.position.y 
        z_pre = source_pose.position.z + GRIPPER_OFFSET_Z + (box.width_m/2) + 0.2 
        # z_pre = source_pose.position.z + GRIPPER_OFFSET_Z + (box.height_m/2) + 0.2

        self.get_logger().info(f'z pre grasp: {z_pre}')
        ik_result = self.ik_planner.compute_ik(self.joint_state, x_pre, y_pre, z_pre, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for pre grasp")
            return False
        self.job_queue.append(ik_result)

        # Calculate Source Grasp (Entering the object)
        x_grasp = x_pre
        y_grasp = y_pre
        z_grasp = z_pre - 0.2 - GRASP_OFFSET_Z # move EE GRASP_OFFSET_Z (1.5cm) into the cube:
        ik_result = self.ik_planner.compute_ik(ik_result, x_grasp, y_grasp, z_grasp, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for post grasp")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # STEP 2: grab, lift

        # Grab
        self.job_queue.append('toggle_grip')

        # Lift back up to pre-grasp position:
        x_lift = x_pre
        y_lift = y_pre
        # z_lift = z_pre
        z_lift = target_pose.position.z + GRIPPER_OFFSET_Z + 0.2 
        
        ik_result = self.ik_planner.compute_ik(ik_result, x_lift, y_lift, z_lift, qx_src, qy_src, qz_src, qw_src)
        if not ik_result: 
            self.get_logger().error("IK failed for lift")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # STEP 3: move to target, and place
        # Position above place location before lowering to place location
        x_pre_place = target_pose.position.x + 0.01 # FIXME why is this offset needed? Is it a box marker issue or a planning issue?
        y_pre_place = target_pose.position.y
        z_pre_place = target_pose.position.z + GRIPPER_OFFSET_Z + 0.2
        
        # ik_result = self.ik_planner.compute_ik(ik_result, x_pre_place, y_pre_place, z_pre_place, qx_dst, qy_dst, qz_dst, qw_dst)
        # self.get_logger().info(f'z pre place: {z_pre_place}')
        ik_result = self.ik_planner.compute_ik(ik_result, x_pre_place, y_pre_place, z_pre_place)
        if not ik_result: 
            self.get_logger().error("IK failed for above place")
            return False
        self.job_queue.append(ik_result)
        
        # Drop to final place height
        x_place = x_pre_place
        y_place = y_pre_place
        z_place = z_pre_place + box.height_m - 0.2 - GRASP_OFFSET_Z + 0.005 
        # NOTE: GRASP_OFFSET_Z for the gripper being inside the cube, 1cm for safety 
        # This should solve the suspiciously high release height!
        
        # ik_result = self.ik_planner.compute_ik(ik_result, x_place, y_place, z_place - 0.1, qx_dst, qy_dst, qz_dst, qw_dst)
        self.get_logger().info(f'target_final_z: {target_pose.position.z}')
        ik_result = self.ik_planner.compute_ik(ik_result, x_place, y_place, z_place)                                                    
        if not ik_result: 
            self.get_logger().error("IK failed for place")
            return False
        self.job_queue.append(ik_result)
        
        # -----------------------------------------------------------
        # STEP 5: release
        self.job_queue.append('toggle_grip')
        
        # -----------------------------------------------------------
        # STEP 6: back out of box to the position in step 3
        ik_result = self.ik_planner.compute_ik(ik_result, x_pre_place, y_pre_place, z_pre_place)
        if not ik_result: 
            self.get_logger().error("IK failed for retreat")
            return False
        self.job_queue.append(ik_result)

        # -----------------------------------------------------------
        # STEP 7: re-open gripper
        self.job_queue.append('toggle_grip')

        self.joint_state = ik_result
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
            time.sleep(0.5)
            self.execute_jobs() 
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

    def calculate_final_pose(self, box_info: tuple, bin_pose, bin) -> Pose:
        # For changing from bin-frame coor to base-link frame
        id, name, fragility, z_base, z_top, x, y = box_info
        
        # box = get_object_by_id(id)
        # pose = Pose()
        
        # # pose.header.frame_id = bin_tf.child_frame_id
        # # pose.header.time = rclpy.time.Time()
        
        # pose.orientation.x = 0.0
        # pose.orientation.y = 0.0
        # pose.orientation.z = 0.0
        # pose.orientation.w = 1.0
        
        # # pose.position.x = x + (box.width_m / 2.0)
        # # pose.position.y = y - (box.length_m / 2.0)
        # # pose.position.z = z_base + (box.height_m / 2.0)

        # pose.position.x = float(x)
        # pose.position.y = float(y)
        # pose.position.z = float(z_base)
        
        # return do_transform_pose(pose, bin_tf)

        self.get_logger().info(f'calc final pose x: {x}, y: {y}')

        un_grid_x = x * bin.resolution
        un_grid_y = y * bin.resolution
        un_grid_z = z_base * bin.resolution

        pose = Pose()
        pose.position.x = float(un_grid_x + bin_pose.position.x) 
        pose.position.y = float(un_grid_y + bin_pose.position.y)
        pose.position.z = float(un_grid_z + bin_pose.position.z)

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()