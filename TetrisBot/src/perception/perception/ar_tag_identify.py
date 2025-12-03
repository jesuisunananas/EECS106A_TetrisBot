import rclpy
from rclpy.node import Node
import tf2_ros
from ros2_aruco_interfaces.msg import ArucoMarkers
# import consolidated marker descriptions as a single source of truth
from shared_things import *
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import numpy as np

from table import average_table_pose

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
        ---
        std_msgs/Header header

        int64[] marker_ids
        geometry_msgs/Pose[] poses
        ---
    '''
    
    # NOTE: the object_dict stuff are all moved to shared_things.aruco_constants1
    # Object descriptions moved to ros2_aruco package
    # object_dict = MARKER_ID_DESCRIPTIONS

    def __init__(self):
        super().__init__("ar_tag_identification_node")

        # Base Marker parameter (set via launch file)
        self.declare_parameter('base_marker', 7)
        self.base_marker = self.get_parameter('base_marker').value

        # "aruco_markers" subscriber
        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10
        )

        # Moveit planning scene
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /apply_planning_scene")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # Iterates through marker ids, identifies ids and stores poses in dictionary
        source_frame = "camera_depth_optical_frame" 
        target_frame = "base_link"

        marker_ids = msg.marker_ids.tolist()
        
        # List to collect all collision objects for this frame
        collision_objects_batch = []
        table_poses = []
        table_item = None

        # self.clear_planning_scene()
        for id in marker_ids:
            if id != self.base_marker:
                    
                # item = TagIdentification.object_dict.get(id)
                item = get_object_by_id(id) # used method defined in shared_things.aruco_constants instead!

                if item is None: 
                    self.get_logger().info(f"item {id} is unknown to society, skip~")
                    continue

                # organise the item based on its id
                try:
                    # Transforms the marker ID to baselink?
                    tf = self.tf_buffer.lookup_transform(target_frame, f'ar_marker_{id}', rclpy.time.Time())
                    pose = Pose()
                    
                    offset = self.offset_centre(item, tf) # add offset to the box to adjust for the AR marker placement
                    pose.position.x = tf.transform.translation.x + offset[0]
                    pose.position.y = tf.transform.translation.y + offset[1]
                    pose.position.z = tf.transform.translation.z + offset[2]
                    pose.orientation = tf.transform.rotation
                    
                    # Check if the ID corresponds to a table, and if the corresponding object is not yet placed
                    if is_table(id):
                        # if item.placed == False:
                        self.get_logger().info(f"The item {id} forms a table")
                        table_poses.append(pose)
                        if table_item is None: table_item = item
                            
                        continue
                    
                    self.get_logger().info(f"posing {item.name}")
                    
                    # Create the object and add to batch instead of sending immediately
                    obj = self.create_collision_object(item, pose)
                    if obj:
                        collision_objects_batch.append(obj)
                
                except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")
            
                # For testing purposes, not needed for actual functionality
                self.get_logger().info(f"Item: {item}")
        
        # Placing the table (only if it is not empty):
        if table_poses and table_item:
            table_item.placed = True # uncomment the one in the for loop if you want to lock the table collision object after placement
            self.get_logger().info(f"Placing Table")
            table_pose = average_table_pose(table_poses)
            table_obj = self.create_collision_object(table_item, table_pose)
            if table_obj:
                collision_objects_batch.append(table_obj)
        
        # Send all updates in one service call. 
        # Apparently this is better for efficiency? hope it doesn't break.
        if collision_objects_batch:
            self.publish_collision_batch(collision_objects_batch)
    
    def create_collision_object(self, item, pose):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        try:
            box.dimensions = [item.length, item.width, item.height]
        except:
            self.get_logger().warn(f"cannot add collison object! item as no dimension attributes!")
            return None
        
        # box.dimensions = [3.0,3.0,3.0]
        # box_pose = Pose()
        # box_pose.position.x = 1.0
        # box_pose.position.y = 1.0
        # box_pose.position.z = 1.0
        # box_pose.orientation.w = 1.0

        coll_obj = CollisionObject()
        coll_obj.header.frame_id = 'base_link'
        coll_obj.id = str(item.id)
        coll_obj.primitives = [box]
        coll_obj.primitive_poses = [pose]
        coll_obj.operation = CollisionObject.ADD
        return coll_obj

    def publish_collision_batch(self, collision_objects):
        scene_msg = PlanningScene()
        scene_msg.world.collision_objects = collision_objects
        scene_msg.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg

        self.get_logger().info('Calling service (batch)')
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

    # def clear_planning_scene(self):
    #     with self.planning_scene_monitor.read_write() as scene:
    #         scene.remove_all_collision_objects()
    #         scene.current_state.update()

    def offset_centre(self, item, tf):
        r = R.from_quat([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ])
        
        local_offset = np.array([0.0, 0.0, -item.height / 2.0])
        
        return r.apply(local_offset)

def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()