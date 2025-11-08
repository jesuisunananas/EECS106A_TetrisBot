#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose

class DraggablePlaneNode(Node):
    def __init__(self):
        super().__init__("draggable_plane_node")

        self.declare_parameter("frame_id", "camera_depth_optical_frame")
        self.declare_parameter("plane_size_x", 1.0)
        self.declare_parameter("plane_size_y", 1.0)
        self.declare_parameter("plane_thickness", 0.01)

        frame_id = self.get_parameter("frame_id").value
        sx = float(self.get_parameter("plane_size_x").value)
        sy = float(self.get_parameter("plane_size_y").value)
        sz = float(self.get_parameter("plane_thickness").value)

        self.pose_pub = self.create_publisher(PoseStamped, "draggable_plane/pose", 10)
        self.create_timer(0.01, self.pub_pose)
        self.create_timer(0.1, self.pose_to_plane_coefficients)

        self.server = InteractiveMarkerServer(self, "draggable_plane")

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = frame_id
        self.int_marker.name = "plane"
        self.int_marker.description = "Drag/Rotate the plane"
        self.int_marker.scale = max(sx, sy) * 1.5

        # Plane visualization
        plane_marker = Marker()
        plane_marker.type = Marker.CUBE
        plane_marker.scale.x = sx
        plane_marker.scale.y = sy
        plane_marker.scale.z = sz
        plane_marker.color.r = 0.2
        plane_marker.color.g = 0.8
        plane_marker.color.b = 0.9
        plane_marker.color.a = 0.5

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(plane_marker)
        self.int_marker.controls.append(vis_ctrl)

        # === Movement controls along each axis ===
        self.add_axis_control(self.int_marker, "move_x", InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0)
        self.add_axis_control(self.int_marker, "move_y", InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0)
        self.add_axis_control(self.int_marker, "move_z", InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0)

        # === Rotation controls around each axis ===
        self.add_axis_control(self.int_marker, "rotate_x", InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0)
        self.add_axis_control(self.int_marker, "rotate_y", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0)
        self.add_axis_control(self.int_marker, "rotate_z", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0)

        self.server.insert(self.int_marker, feedback_callback=self.feedback_cb)
        self.server.applyChanges()
        self.get_logger().info("Interactive plane with full 6-DOF control is ready!")

    def add_axis_control(self, marker, name, mode, x, y, z):
        ctrl = InteractiveMarkerControl()
        ctrl.name = name
        ctrl.interaction_mode = mode
        ctrl.orientation.w = 1.0
        ctrl.orientation.x = x
        ctrl.orientation.y = y
        ctrl.orientation.z = z
        marker.controls.append(ctrl)

    def feedback_cb(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            msg = PoseStamped()
            msg.header = feedback.header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = feedback.pose
            self.pose_pub.publish(msg)

    def pub_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = self.int_marker.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self.int_marker.pose
        self.pose_pub.publish(msg)

    def pose_to_plane_coefficients(self):
        pose = self.int_marker.pose
        # Extract position
        position = np.array([pose.position.x, pose.position.y, pose.position.z])

        # Extract quaternion and convert to rotation matrix
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation = R.from_quat(quat)
        rot_matrix = rotation.as_matrix()  # 3x3

        # Plane normal is Z-axis of the local frame rotated into world frame
        local_z_axis = np.array([0, 0, 1])
        world_normal = rot_matrix @ local_z_axis  # Matrix-vector multiply

        # Plane coefficients
        A, B, C = world_normal
        D = -np.dot(world_normal, position)

        self.get_logger().info(f"A: {A}, B: {B}, C: {C}, D: {D}")

def main():
    rclpy.init()
    node = DraggablePlaneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# A: 0.010304802867131112, B: 0.9970237525906941, C: 0.07640319239298748, D: -0.07215145360173988