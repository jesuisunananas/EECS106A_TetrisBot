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
)
# I've included all the marker IDs here! 
# They are in the ros2_aruco package

# from moveit.planning import MoveItPy

# from geometry_msgs.msg import Pose
# from moveit_msgs.msg import CollisionObject
# from shape_msgs.msg import SolidPrimitive

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose


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
        # self.panda = MoveItPy(node_name="moveit_py_planning_scene")
        # self.panda_arm = panda.get_planning_component("panda_arm")
        # self.planning_scene_monitor = panda.get_planning_scene_monitor()
        # logger.info("MoveItPy instance created")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # Iterates through marker ids, identifies ids and stores poses in dictionary
        source_frame = "camera_depth_optical_frame" 
        target_frame = "base_link"

        # try:
        #     tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        # except tf2_ros.TransformException as ex:
        #     self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")
        #     return None

        marker_ids = msg.marker_ids.tolist()
        new_found_boxes = {}
        new_found_bins = {}
        new_unknown_items = {}
        # self.clear_planning_scene()
        for i, id in enumerate(marker_ids):
            if id != self.base_marker:
                item = TagIdentification.object_dict.get(id)

                # organise the item based on its id
                if item and id in BOX_MARKER_IDS:
                    try:
                        tf = self.tf_buffer.lookup_transform(f'ar_marker_{id}', source_frame, rclpy.time.Time())
                        # self.get_logger().info("doing pose")
                        new_found_boxes[item] = tf
                    except tf2_ros.TransformException as ex:
                        self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")

                    # self.get_logger().info("doing pose")
                    # new_found_boxes[item] = tf
                    # new_found_boxes[item] = do_transform_pose(msg.poses[i], tf)
                    # self.add_collision_object(msg.poses[i], 'box')
                elif item and id in BIN_MARKER_IDS:
                    try:
                        tf = self.tf_buffer.lookup_transform(f'ar_marker_{id}', source_frame, rclpy.time.Time())
                        # self.get_logger().info("doing pose")
                        new_found_bins[item] = tf
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
                        new_unknown_items[item] = tf
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
                print(self.found_boxes[item])
    
    def add_collision_object(self, pose, obj_type):
        """Helper function that adds collision object to the planning scene."""
        object_dimensions = [
            (0.1, 0.4, 0.1),
            (0.2, 0.4, 0.2),
        ]

        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.id = "boxes"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = object_dimensions[0] if obj_type == 'box' else object_dimensions[1]

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()  # Important to ensure the scene is updated

    def clear_planning_scene(self):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()


def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()