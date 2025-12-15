#from ultralytics import YOLO
import random
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

#from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs


from box_bin_msgs.msg import BoxBin
from shared_things import BOXES, BINS


def depth_to_points(depth, mask, K):
    """Convert masked depth image to Nx3 point cloud (camera frame)."""
    ys, xs = np.where(mask)
    z = depth[ys, xs] * 0.001  # mm → meters
    valid = z > 0
    xs, ys, z = xs[valid], ys[valid], z[valid]

    x = (xs - K[0, 2]) * z / K[0, 0]
    y = (ys - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1)


def estimate_yaw_pca(points):
    """Estimate yaw from PCA in XY plane."""
    pts = points[:, :2]
    cov = np.cov(pts.T)
    _, vecs = np.linalg.eigh(cov)
    yaw = np.arctan2(vecs[1, 1], vecs[0, 1])
    return yaw

class RGBDBoxPoseNode(Node):
    def __init__(self):
        super().__init__("rgbd_box_pose_node")

        #self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, 10
        )

        self.box_bin_pub = self.create_publisher(BoxBin, "/box_bin", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #self.model = YOLO("TetrisBot/src/perception/perception/yolov8n-seg.pt")

        self.rgb = None
        self.depth = None
        self.K = None
        self.camera_frame = None

        self.get_logger().info("RGB-D Box Pose Node initialized.")
    
    def info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def rgb_callback(self, msg: Image):
        #self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    def depth_callback(self, msg: Image):
        #self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.try_process()
    
    def try_process(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return
        print("hello")


def main(args=None):
    rclpy.init(args=args)
    node = RGBDBoxPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()