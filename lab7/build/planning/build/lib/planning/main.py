# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped 
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_pub = self.create_subscription(PointStamped, '/cube_pose_in_base', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
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

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def cube_callback(self, cube_pose):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose

        # -----------------------------------------------------------
        # TODO: In the following section you will add joint angles to the job queue. 
        # Entries of the job queue should be of type either JointState or String('toggle_grip')
        # Think about you will leverage the IK planner to get joint configurations for the cube grasping task.
        # To understand how the queue works, refer to the execute_jobs() function below.
        # -----------------------------------------------------------


        # NOTE extract cube position (base_link frame), Ziteng believe we need it, not sure
        cx = float(cube_pose.point.x)
        cy = float(cube_pose.point.y)
        cz = float(cube_pose.point.z)

        # 1) Move to Pre-Grasp Position (gripper above the cube)
        '''
        Use the following offsets for pre-grasp position:
        x offset: 0.0
        y offset: -0.035 (Think back to lab 5, why is this needed?)
        z offset: +0.185 (to be above the cube by accounting for gripper length)
        '''
        pre_dx, pre_dy, pre_dz = 0.0, -0.035, 0.185
        grasp_dz_min = 0.16

        def _append_ik_target(x, y, z, note):
            js = self.ik_planner.compute_ik(self.joint_state, x, y, z)
            if js is None:
                self.get_logger().error(f"IK failed for {note} at ({x:.3f}, {y:.3f}, {z:.3f})")
                return False
            self.job_queue.append(js)
            self.get_logger().info(f"Queued {note} IK.")
            return True

        pre_x = cx + pre_dx
        pre_y = cy + pre_dy
        pre_z = cz + pre_dz
        if not _append_ik_target(pre_x, pre_y, pre_z, "pre-grasp"):
            return

        # 2) Move to Grasp Position (lower the gripper to the cube)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16. 
        '''
        grasp_x = cx + pre_dx
        grasp_y = cy + pre_dy
        grasp_z = max(cz + grasp_dz_min, cz + 0.16)
        if not _append_ik_target(grasp_x, grasp_y, grasp_z, "grasp"):
            return

        # 3) Close the gripper. See job_queue entries defined in init above for how to add this action.
        self.job_queue.append('toggle_grip')
        
        # 4) Move back to Pre-Grasp Position
        if not _append_ik_target(pre_x, pre_y, pre_z, "retreat-to-pre-grasp"):
            return

        # 5) Move to release Position
        '''
        We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
        Which offset will you change to achieve this and in what direction?
        '''
        place_dx = 0.4
        place_x = cx + place_dx
        place_y = cy + pre_dy
        place_z = grasp_z
        if not _append_ik_target(place_x, place_y, place_z, "place"):
            return

        # 6) Release the gripper
        self.job_queue.append('toggle_grip')

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
