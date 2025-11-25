import rclpy
from rclpy.node import Node
import tf2_ros
from ros2_aruco_interfaces.msg import ArucoMarkers
# import consolidated marker descriptions as a single source of truth
from ros2_aruco.aruco_constants import (
    BOX_MARKER_IDS,
    BIN_MARKER_IDS,
    BOX_MARKER_SIZE,
    BIN_MARKER_SIZE,
    DEFAULT_MARKER_SIZE,
    MARKER_ID_DESCRIPTIONS,
    MARKER_ID,
)
# I've included all the marker IDs here! 
# They are in the ros2_aruco package

from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from packing.box import Box, Bin 


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
    
    # Object descriptions moved to ros2_aruco package
    object_dict = MARKER_ID_DESCRIPTIONS

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
        # self.clear_planning_scene()
        for i, id in enumerate(marker_ids):
            if id != self.base_marker:
                # item = TagIdentification.object_dict.get(id)
                item = MARKER_ID.get(id)

                # organise the item based on its id
                if isinstance(item, Box):
                    try:
                        tf = self.tf_buffer.lookup_transform(f'ar_marker_{id}', source_frame, rclpy.time.Time())
                        # self.get_logger().info("doing pose")
                        item.pose = tf
                        new_found_boxes[item.name] = item
                        self.add_collision_object(item)
                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")

                    # self.get_logger().info("doing pose")
                    # new_found_boxes[item] = tf
                    # new_found_boxes[item] = do_transform_pose(msg.poses[i], tf)
                    # self.add_collision_object(msg.poses[i], 'box')
                elif isinstance(item, Bin):
                    try:
                        tf = self.tf_buffer.lookup_transform(f'ar_marker_{id}', source_frame, rclpy.time.Time())
                        # self.get_logger().info("doing pose")
                        item.pose = tf
                        new_found_bins[item.name] = item
                        self.add_collision_object(item)
                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")

                    # self.get_logger().info("doing pose")
                    # new_found_bins[item] = tf
                    # new_found_bins[item] = do_transform_pose(msg.poses[i], tf)
                    # self.add_collision_object(msg.poses[i], 'bin')
                else:
                    try:
                        tf = self.tf_buffer.lookup_transform(f'ar_marker_{id}', source_frame, rclpy.time.Time())
                        # self.get_logger().info("doing pose")
                        new_unknown_items[f'unknown_item_{i}'] = tf
                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")
                    # self.get_logger().info("doing pose")
                    # new_unknown_items[item] = tf
                    # new_unknown_items[item] = do_transform_pose(msg.poses[i], tf)

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
    
    def add_collision_object(self, item):
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [item.length, item.width, item.height]
        # box.dimensions = [3.0,3.0,3.0]
        # box_pose = Pose()
        # box_pose.position.x = 1.0
        # box_pose.position.y = 1.0
        # box_pose.position.z = 1.0
        # box_pose.orientation.w = 1.0


        coll_obj = CollisionObject()
        coll_obj.header.frame_id = 'base_link'
        coll_obj.id = item.id
        coll_obj.primitives = [box]
        coll_obj.primitive_poses = [item.pose]
        coll_obj.operation = CollisionObject.ADD

        scene_msg = PlanningScene()
        scene_msg.world.collision_objects.append(coll_obj)
        scene_msg.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg

        self.get_logger().info('Calling service')
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