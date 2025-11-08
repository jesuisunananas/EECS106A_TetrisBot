import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
from scipy.spatial.transform import Rotation as R

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_in_base_pub = self.create_publisher(PointStamped, '/cube_pose_in_base', 10) # Please ensure this is filled

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        transformed = self.transform_cube_pose(msg)
        if transformed is None:
            return
        self.cube_pose = transformed
        self.cube_in_base_pub.publish(transformed)
        self.get_logger().info(
            f"Cube (base_link): x={transformed.point.x:.3f}, "
            f"y={transformed.point.y:.3f}, z={transformed.point.z:.3f}"
        )

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            PointStamped: point in base_link_frame in form [x, y, z]
        """

        # ------------------------
        #TODO: Add your code here!
        # ------------------------
        source_frame = "camera_depth_optical_frame" 
        target_frame = "base_link"

        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({source_frame} -> {target_frame}): {ex}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.array([t.x, t.y, t.z], dtype=float)

        p_src = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        p_tgt = R_mat @ p_src + T

        out = PointStamped()
        out.header.frame_id = target_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.point.x, out.point.y, out.point.z = map(float, p_tgt)
        return out

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
