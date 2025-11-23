import rclpy
from rclpy.node import Node
import tf2_ros
from ros2_aruco_interfaces.msg import ArucoMarkers
# import consolidated marker descriptions as a single source of truth
from ros2_aruco.ros2_aruco.aruco_constants import (
    BOX_MARKER_IDS,
    BIN_MARKER_IDS,
    BOX_MARKER_SIZE,
    BIN_MARKER_SIZE,
    DEFAULT_MARKER_SIZE,
    MARKER_ID_DESCRIPTIONS,
)
# I've included all the marker IDs here! 
# They are in the ros2_aruco package

class TagIdentification(Node):
    '''
    Notes;
    - current aruco dictionary is 5x5 250 (can go up to 250 codes)
        - theres a tag generator in ros2_aruco folder :D
    - can technically work with very small tags, but seems like pose is unstable,
        seems stablish around 20mm which is roughly size of bottlecap
    - uh colcon build scares me, idk why it yells at me when I build in project 
        folder instead of in normal lab folder
        - something with how ArucoMarkers msg was built I think
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
        self.timer = self.create_timer(0.5, self.print_markers)

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # Iterates through marker ids, identifies ids and stores poses in dictionary
        marker_ids = msg.marker_ids.tolist()
        for i, id in enumerate(marker_ids):
            if id != self.base_marker:
                item = TagIdentification.object_dict.get(id)
                
                # organise the item based on its id
                if item and id in BOX_MARKER_IDS:
                    self.found_boxes[item] = msg.poses[i]
                elif item and id in BIN_MARKER_IDS:
                    self.found_bins[item] = msg.poses[i]
                else:
                    self.unknown_item[item] = msg.poses[i]
    
    # For testing purposes, not needed for actual functionality
    def print_markers(self):
        if self.items:
            for item in self.items.keys():
                self.get_logger().info(f"Item: {item}")
                print(self.items[item])



def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()