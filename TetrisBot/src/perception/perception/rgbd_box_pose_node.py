#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs

from ultralytics import YOLO

from box_bin_msgs.msg import BoxBin
from shared_things import BOXES, BINS

# ---------------------------
# Utility functions
# ---------------------------

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


# ---------------------------
# Main Node
# ---------------------------

class RGBDBoxPoseNode(Node):
    def __init__(self):
        super().__init__("rgbd_box_pose_node")

        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, 10
        )

        # Publisher
        self.box_bin_pub = self.create_publisher(BoxBin, "/box_bin", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLOv8 segmentation
        self.model = YOLO("yolov8n-seg.pt")

        # State
        self.rgb = None
        self.depth = None
        self.K = None
        self.camera_frame = None

        self.get_logger().info("RGB-D Box Pose Node initialized.")

    # ---------------------------
    # Callbacks
    # ---------------------------

    def info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def rgb_callback(self, msg: Image):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.try_process()

    def depth_callback(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.try_process()

    # ---------------------------
    # Main processing
    # ---------------------------

    def try_process(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return

        results = self.model(self.rgb, verbose=False)[0]
        if results.masks is None:
            return

        box_ids = []
        box_poses = []

        for i, mask in enumerate(results.masks.data):
            class_id = int(results.boxes.cls[i])
            conf = float(results.boxes.conf[i])

            if conf < 0.6:
                continue

            # Map class → Box object
            if class_id not in BOXES:
                continue

            box = BOXES[class_id]

            mask_np = mask.cpu().numpy().astype(bool)
            points = depth_to_points(self.depth, mask_np, self.K)

            if len(points) < 100:
                continue

            # Position (robust)
            xyz = np.median(points, axis=0)

            # Orientation (yaw only)
            yaw = estimate_yaw_pca(points)

            # Build pose in camera frame
            pose_cam = PoseStamped()
            pose_cam.header.frame_id = self.camera_frame
            pose_cam.header.stamp = self.get_clock().now().to_msg()

            pose_cam.pose.position.x = float(xyz[0])
            pose_cam.pose.position.y = float(xyz[1])
            pose_cam.pose.position.z = float(xyz[2])

            q = tf2_geometry_msgs.transformations.quaternion_from_euler(
                np.pi, 0.0, yaw
            )
            pose_cam.pose.orientation.x = q[0]
            pose_cam.pose.orientation.y = q[1]
            pose_cam.pose.orientation.z = q[2]
            pose_cam.pose.orientation.w = q[3]

            # Transform to base_link
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                pose_base = tf2_geometry_msgs.do_transform_pose(
                    pose_cam.pose, tf
                )
            except Exception as e:
                self.get_logger().warn(f"TF failed: {e}")
                continue

            box_ids.append(box.id)
            box_poses.append(pose_base)

        if not box_ids:
            return

        # Bin pose (still from ArUco for now)
        bin_id = list(BINS.keys())[0]
        bin = BINS[bin_id]

        bin_pose = Pose()
        bin_pose.position.x = 0.0
        bin_pose.position.y = 0.0
        bin_pose.position.z = 0.0
        bin_pose.orientation.w = 1.0

        msg = BoxBin()
        msg.box_ids = box_ids
        msg.box_poses = box_poses
        msg.bin_ids = [bin_id]
        msg.bin_poses = [bin_pose]

        self.box_bin_pub.publish(msg)


# ---------------------------
# Entry point
# ---------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RGBDBoxPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
