#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import scipy 
from scipy.spatial.transform import Rotation as R

import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        #AR Marker parameter
        self.declare_parameter('ar_marker', "8")
        self.ar_marker = self.get_parameter('ar_marker').value

        # Homogeneous transform G_ar->base
        G = np.array([
            [-1, 0, 0, 0.0],
            [ 0, 0, 1, 0.16],
            [ 0, 1, 0, -0.13],
            [ 0, 0, 0, 1.0]
        ])

        # Create TransformStamped
        self.transform = TransformStamped()
        # transformation = scipy.RigidTransform.from_matrix(G)
        # ---------------------------
        # TODO: Fill out TransformStamped message

        R_mat = G[:3, :3]
        t = G[:3, 3]
        x, y, z, w = R.from_matrix(R_mat).as_quat()

        self.transform.header.frame_id = "ar_marker_" + self.ar_marker
        self.transform.child_frame_id = "base_link"
        self.transform.transform.translation.x = float(t[0])
        self.transform.transform.translation.y = float(t[1])
        self.transform.transform.translation.z = float(t[2])
        
        self.transform.transform.rotation.x = float(x)
        self.transform.transform.rotation.y = float(y)
        self.transform.transform.rotation.z = float(z)
        self.transform.transform.rotation.w = float(w)

        # --------------------------

        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()