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

from perception.table import average_table_pose
from perception.mesh import get_collision_mesh

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

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # The frame the camera is reporting in (usually optical_frame)
        source_frame = msg.header.frame_id.lstrip('/')
        target_frame = "base_link"

        # List to collect all collision objects for this frame
        collision_objects_batch = []
        table_poses = []
        table_item = None
        
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

                    # # apply transform to this PoseStamped
                    transform_timeout = rclpy.duration.Duration(seconds=0.1)
                    # transformed_pose = self.tf_buffer.transform(source_pose, target_frame, timeout=transform_timeout)
                    tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=transform_timeout)
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(source_pose.pose, tf)
                    
                    pose = transformed_pose

                    if is_box(id):
                        # self.get_logger().info(f"The item {id} is a box")
                        box_ids.append(id)
                        box_poses.append(pose)
                        
                    elif is_bin(id):
                        # self.get_logger().info(f"The item {id} is a bin")
                        bin_ids.append(id)
                        bin_poses.append(pose)
                        
                    elif is_table(id):
                        # self.get_logger().info(f"The item {id} forms a table")
                        table_poses.append(pose)
                        self.get_logger().info(f'Table z: {[pose.position.z]}')
                        if table_item is None: table_item = item
                        continue

                    offset = self.offset_centre(item, pose.orientation) 
                    
                    pose.position.x += offset[0]
                    pose.position.y += offset[1]
                    pose.position.z += offset[2]
                    
                    self.get_logger().info(f"posing {item.name}, at position ({pose.position.x}, {pose.position.y}, {pose.position.z})")
                    obj = self.create_collision_object(item, pose)
                    
                    # Create the object and add to batch
                    if obj:
                        collision_objects_batch.append(obj)
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    self.get_logger().warn(f"TF transform failed ({source_frame} -> {target_frame}): {ex}")
            
        # publish categorised bins and boxes
        box_bins = BoxBin()
        box_bins.box_ids = box_ids
        box_bins.box_poses = box_poses
        box_bins.bin_ids = bin_ids
        box_bins.bin_poses = bin_poses
        self.box_bin_pub.publish(box_bins)
            
        # Placing the table (only if it is not empty):
        if table_poses and table_item:
            # table_item.placed = True 
            # self.get_logger().info(f"Placing Table")
            table_pose = average_table_pose(table_item, table_poses)
            table_obj = self.create_collision_object(table_item, table_pose)
            if table_obj:
                collision_objects_batch.append(table_obj)
        
        # Send all updates in one service call. 
        if collision_objects_batch:
            self.publish_collision_batch(collision_objects_batch)
    
    def create_collision_object(self, item, pose):
        
        coll_obj = CollisionObject()
        
        mesh_filepath = None
        if isinstance(item.id, int):
            mesh_filepath = get_mesh_path(item.id)
        if not mesh_filepath:
            try:
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [item.length, item.width, item.height]
                coll_obj.primitives = [box]
                coll_obj.primitive_poses = [pose]
            except:
                self.get_logger().warn(f"cannot add collison object! item as no dimension attributes nor mesh path!")
                return None
        else:
            mesh = get_collision_mesh(self.get_logger(), mesh_filepath)
            if not mesh:
                self.get_logger().warn(f"cannot add collison mesh!")
                return None
            coll_obj.meshes = [mesh]
            coll_obj.mesh_poses = [pose]
        
        coll_obj.header.frame_id = 'base_link'
        coll_obj.id = str(item.id)
        
        coll_obj.operation = CollisionObject.ADD
        return coll_obj

    def publish_collision_batch(self, collision_objects):
        scene_msg = PlanningScene()
        scene_msg.world.collision_objects = collision_objects
        scene_msg.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg

        # self.get_logger().info('Calling service (batch)')
        future = self.scene_cli.call_async(req)
        future.add_done_callback(self.collision_response_callback)

    def collision_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn('Failed to add collision objects')
        except Exception as e:
            self.get_logger().warn(f'Fail to add collision objects: {e}')

    def offset_centre(self, item, orientation_q):
        """
        Calculates the offset in the world frame based on the object's orientation.
        """
        r = R.from_quat([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        # Offset the box centre from the marker:
        local_offset = np.array([0.0, 0.0, -item.height / 2.0])
        
        # Rotate that offset vector to align with the object's current orientation in world
        return r.apply(local_offset)

def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()