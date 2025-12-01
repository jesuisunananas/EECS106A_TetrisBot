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
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from collision_objects import average_table_pose

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

        self.found_boxes = {}
        self.found_bins = {}
        self.unknown_item = {}

        # For testing purposes, not needed for actual functionality
        self.timer = self.create_timer(1, self.print_markers)

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
        new_found_boxes = {}
        new_found_bins = {}
        new_unknown_items = {}

        # List to collect all collision objects for this frame
        collision_objects_batch = []

        # self.clear_planning_scene()
        for i, id in enumerate(marker_ids):
            if id != self.base_marker:
                # item = TagIdentification.object_dict.get(id)
                item = get_object_by_id(id) # used method defined in shared_things.aruco_constants instead!

                if item is None: continue

                # organise the item based on its id
                try:
                    # Transforms the marker ID to baselink?
                    tf = self.tf_buffer.lookup_transform(target_frame, f'ar_marker_{id}', rclpy.time.Time())
                    pose = Pose()
                    pose.position.x = tf.transform.translation.x
                    pose.position.y = tf.transform.translation.y
                    pose.position.z = tf.transform.translation.z
                    pose.orientation = tf.transform.rotation

                    # item.pose = tf
                    self.get_logger().info(f"posing {item.name}")
                    
                    # Create the object and add to batch instead of sending immediately
                    obj = self.create_collision_object(item, pose)
                    if obj:
                        collision_objects_batch.append(obj)
                
                except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")
                
                if is_box(id):
                    self.get_logger().info(f"item {id} is a box")
                    new_found_boxes[item.name] = item
                elif is_bin(id):
                    self.get_logger().info(f"item {id} is a bin")
                    new_found_bins[item.name] = item
                else:
                    self.get_logger().info(f"item {id} is neither a box nor a bin")
                    new_unknown_items[f'unknown_item_{i}'] = tf
        
        # Send all updates in one service call
        if collision_objects_batch:
            self.publish_collision_batch(collision_objects_batch)

        self.found_bins = new_found_bins
        self.found_boxes = new_found_boxes
        self.unknown_item = new_unknown_items
    
    # For testing purposes, not needed for actual functionality
    def print_markers(self):
        if self.found_boxes:
            self.get_logger().info("------------------------------------------")
            for item in self.found_boxes.keys():
                self.get_logger().info(f"Item: {item}")
                # print(self.found_boxes[item])
    
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


def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()