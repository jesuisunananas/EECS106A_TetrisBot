import rclpy
from rclpy.node import Node
import tf2_ros
from ros2_aruco_interfaces.msg import ArucoMarkers

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
    object_dict = {
        15: 'mmm bread'
    }

    def __init__(self):
        super().__init__("ar_tag_identification_node")

        #Base Marker parameter, TODO: put parameter in launch file
        self.declare_parameter('base_marker', 7)
        self.base_marker = self.get_parameter('base_marker').value

        # "aruco_markers" subscriber
        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10
        )

        self.items = {}

        # For testing purposes, not needed for actual functionality
        self.timer = self.create_timer(0.5, self.print_markers)

    def aruco_marker_callback(self, msg: ArucoMarkers):
        # Iterates through marker ids, identifies ids and stores poses in dictionary
        marker_ids = msg.marker_ids.tolist()
        for i, id in enumerate(marker_ids):
            if id != self.base_marker: 
                item = TagIdentification.object_dict[id]
                if item:
                    self.items[TagIdentification.object_dict[id]] = msg.poses[i]
    
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