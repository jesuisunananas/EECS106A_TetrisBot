import rclpy
from rclpy.node import Node
import tf2_ros
from ros2_aruco_interfaces.msg import ArucoMarkers
# import yaml

class TagIdentification(Node):
    object_dict = {
        'ar_marker_15': 'mmm bread'
    }

    def __init__(self):
        super().__init__("ar_tag_identification_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.aruco_marker_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10
        )

        self.items = {}

        self.timer = self.create_timer(10, self.print_markers)
        # self.timer = self.create_timer(10, self.get_frames)

    # def get_frames(self):
    #     # Got how get all frames from following:
    #     # https://robotics.stackexchange.com/questions/95228/how-to-get-list-of-all-tf-frames-programatically
        
    #     # frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
    #     # frames_list = list(frames_dict.keys()) 
    #     # ar_markers_list = list(filter(lambda item: 'ar_marker' in item, frames_list))
    #     self.get_logger().info(self.tf_buffer.all_frames_as_yaml())
    def aruco_marker_callback(self, msg: ArucoMarkers):
        marker_ids = msg.marker_ids
        for i, id in enumerate(marker_ids):
            if id != 'ar_marker_7': #CHANGE TO BASE LINK ARUCO TAG
                self.items[TagIdentification.object_dict[id]] = msg.poses[i]
    def print_markers(self):
        if self.items.values:
            for id, item in self.items:
                self.get_logger().info(f"Item: {item}, ID: {id}")



def main(args=None):
    rclpy.init(args=args)
    node = TagIdentification()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()