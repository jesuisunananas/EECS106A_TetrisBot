import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs 
from ros2_aruco_interfaces.msg import ArucoMarkers
# import consolidated marker descriptions as a single source of truth
from shared_things import *
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from box_bin_msgs.msg import BoxBin

class TagIdentification(Node):
    '''
    Notes;
    - current aruco dictionary is 5x5 250 (can go up to 250 codes)
        - theres a tag generator in ros2_aruco folder :D
    - can technically work with very small tags, but seems like pose is unstable,
        seems stablish around 20mm which is roughly size of bottlecap
    - if colcon build yells at you, maybe delete existing build folder
    - aruco_node publishes all markers and marker poses to topic "ar_markers" with 
        message type ArucoMarkers
    '''
    
    def __init__(self):
        super().__init__("ar_tag_identification_node")

        # Base Marker parameter (set via launch file)
        self.declare_parameter('base_marker', 10)
        self.base_marker = self.get_parameter('base_marker').value

        # "aruco_markers" subscriber
        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10
        )
        
        self.box_bin_pub = self.create_publisher(BoxBin, "box_bin", 10)

        # Moveit planning scene
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /apply_planning_scene")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.active_items = set()

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # The frame the camera is reporting in (usually optical_frame)
        source_frame = msg.header.frame_id.lstrip('/')
        self.get_logger().info(f"Source frame is in {source_frame}")
        target_frame = "base_link"

        # for box_bin publisher:
        box_ids = []
        bin_ids = []
        box_poses = []
        bin_poses = []
        
        # Zip the IDs and Poses to simplify code ig
        for id, input_pose in zip(msg.marker_ids, msg.poses):
            if id != self.base_marker:
                item = get_object_by_id(id) 

                if item is None: 
                    self.get_logger().info(f"marker {id} is unknown to society, skip~")
                    continue

                try:
                    # create pose
                    source_pose = PoseStamped()
                    # source_pose.header = msg.header
                    source_pose.header.frame_id = msg.header.frame_id
                    source_pose.header.stamp = self.get_clock().now().to_msg()
                    source_pose.pose = input_pose

                    # apply transform to this PoseStamped
                    transform_timeout = rclpy.duration.Duration(seconds=0.1)
                    # transformed_pose = self.tf_buffer.transform(source_pose, target_frame, timeout=transform_timeout)
                    tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=transform_timeout)
                    tf_pose = tf2_geometry_msgs.do_transform_pose(source_pose.pose, tf)

                    if is_box(id):
                        box_ids.append(id)
                        box_poses.append(tf_pose)

                    elif is_bin(id):                        
                        bin_ids.append(id)
                        bin_poses.append(tf_pose)
                    
                    # self.get_logger().info(f"posing {item.name}, at position ({tf_pose.position.x}, {tf_pose.position.y}, {tf_pose.position.z})")   
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    self.get_logger().warn(f"TF transform failed ({source_frame} -> {target_frame}): {ex}")
            
        # publish categorised bins and boxes
        box_bins = BoxBin()
        box_bins.box_ids = box_ids
        box_bins.box_poses = box_poses
        box_bins.bin_ids = bin_ids
        box_bins.bin_poses = bin_poses
        self.box_bin_pub.publish(box_bins)
        
    def create_offset(self, offset, q_x, q_y, q_z, q_w):
        """
        Calculates the offset in the world frame based on the object's orientation.
        """
        r = R.from_quat([q_x, q_y, q_z, q_w]) # NOTE: don't pass in orientation so prevent mutations

        # Offset the box centre from the marker:
        local_offset = np.array(offset)
        
        # Rotate that offset vector to align with the object's current orientation in world
        return r.apply(local_offset)

def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()