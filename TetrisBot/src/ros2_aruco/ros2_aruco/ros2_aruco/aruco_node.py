"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Published Transforms:
    tf2 transforms from camera frame to each detected marker
    Child frame names: "ar_marker_{marker_id}"

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster
from pkg_resources import parse_version # to handle different CV2 versions :(

from shared_things import *

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")
        
        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=DEFAULT_MARKER_SIZE,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )


        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Set up tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        #------------------------------CV2-Version-Patch-from-Kevin----------------------------------
        if parse_version(cv2.__version__) >= parse_version("4.7.0"):
            # cv2 version is newer:
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        else:
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        #==============================CV2-Version-Patch-from-Kevin==================================

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        #------------------------------CV2-Version-Patch-from-Kevin----------------------------------
        if parse_version(cv2.__version__) >= parse_version("4.7.0"):
            # again, if CV2 version is newer;
            corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
            )
        #------------------------------CV2-Version-Patch-from-Kevin----------------------------------

        # for id in marker_ids:
        #     self.get_logger().info(id)

        if marker_ids is not None:
            # Organize markers with their corresponding sizes and corners
            marker_data = []  # List of (index, marker_id, marker_size, corners)
            bigger_marker_data = []
            small_corners = []
            big_corners = []

            small_ids = []
            big_ids = []
            
            for i, marker_id in enumerate(marker_ids):
                marker_size = get_marker_size(marker_id[0]) # <- new function imported from aruco_constants.py!!
                if marker_size == BIN_MARKER_SIZE:
                    marker_data.append((i, marker_id, marker_size, corners[i]))
                    small_corners.append(corners[i])
                    small_ids.append(marker_id)
                elif marker_size == DEFAULT_MARKER_SIZE:
                    bigger_marker_data.append((i, marker_id, marker_size, corners[i]))
                    big_corners.append(corners[i])
                    big_ids.append(marker_id)
            



            
            # Estimate poses for all markers at once
            # Handle empty arrays to prevent crashes when markers are temporarily lost
            if len(small_corners) > 0:
                if parse_version(cv2.__version__) >= parse_version("4.7.0"):
                    rvecs, tvecs, _ = custom_estimatePoseSingleMarkers(
                        small_corners, BOX_MARKER_SIZE, self.intrinsic_mat, self.distortion
                    )
                elif cv2.__version__ > "4.0.0":
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        small_corners, BOX_MARKER_SIZE, self.intrinsic_mat, self.distortion
                    )
                else:
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                        small_corners, marker_ids, self.intrinsic_mat, self.distortion
                    )
                # Ensure proper shape (N, 1, 3)
                if rvecs.ndim == 0 or len(rvecs) == 0:
                    rvecs = np.empty((0, 1, 3))
                if tvecs.ndim == 0 or len(tvecs) == 0:
                    tvecs = np.empty((0, 1, 3))
            else:
                rvecs = np.empty((0, 1, 3))
                tvecs = np.empty((0, 1, 3))
            
            if len(big_corners) > 0:
                if parse_version(cv2.__version__) >= parse_version("4.7.0"):
                    big_rvecs, big_tvecs, _ = custom_estimatePoseSingleMarkers(
                        big_corners, DEFAULT_MARKER_SIZE, self.intrinsic_mat, self.distortion
                    )
                elif cv2.__version__ > "4.0.0":
                    big_rvecs, big_tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        big_corners, DEFAULT_MARKER_SIZE, self.intrinsic_mat, self.distortion
                    )
                else:
                    big_rvecs, big_tvecs = cv2.aruco.estimatePoseSingleMarkers(
                        big_corners, DEFAULT_MARKER_SIZE, self.intrinsic_mat, self.distortion
                    )
                # Ensure proper shape (N, 1, 3)
                if big_rvecs.ndim == 0 or len(big_rvecs) == 0:
                    big_rvecs = np.empty((0, 1, 3))
                if big_tvecs.ndim == 0 or len(big_tvecs) == 0:
                    big_tvecs = np.empty((0, 1, 3))
            else:
                big_rvecs = np.empty((0, 1, 3))
                big_tvecs = np.empty((0, 1, 3))
            
            # Safely append arrays (both are guaranteed to be 2D now)
            if len(rvecs) > 0 and len(big_rvecs) > 0:
                rvecs = np.append(rvecs, big_rvecs, axis=0)
                tvecs = np.append(tvecs, big_tvecs, axis=0)
            elif len(big_rvecs) > 0:
                rvecs = big_rvecs
                tvecs = big_tvecs
            # If both are empty, rvecs and tvecs remain empty arrays

            small_ids.extend(big_ids)
            marker_ids = small_ids
            
            # Skip processing if no valid poses were computed
            if len(rvecs) == 0 or len(tvecs) == 0 or len(marker_ids) == 0:
                return
            
            # Ensure arrays match marker count
            if len(rvecs) != len(marker_ids) or len(tvecs) != len(marker_ids):
                self.get_logger().warn(f"Mismatch: {len(rvecs)} rvecs, {len(tvecs)} tvecs, {len(marker_ids)} markers")
                return
            
            # Process all markers
            for i, marker_id in enumerate(marker_ids):
                # self.get_logger().info(f"marker detected: {marker_id}")
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

                # Convert rotation matrix to quaternion
                quat = Rotation.from_matrix(rot_matrix[0:3, 0:3]).as_quat()

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                # Publish tf transform for this marker
                transform = TransformStamped()
                transform.header.stamp = img_msg.header.stamp
                if self.camera_frame == "":
                    transform.header.frame_id = "camera1"
                else:
                    transform.header.frame_id = self.camera_frame
                transform.child_frame_id = f"ar_marker_{marker_id[0]}"
                
                transform.transform.translation.x = pose.position.x
                transform.transform.translation.y = pose.position.y
                transform.transform.translation.z = pose.position.z
                transform.transform.rotation.x = pose.orientation.x
                transform.transform.rotation.y = pose.orientation.y
                transform.transform.rotation.z = pose.orientation.z
                transform.transform.rotation.w = pose.orientation.w
                
                self.tf_broadcaster.sendTransform(transform)
                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            # self.get_logger().info(f"markers detected: {markers}")
            # self.get_logger().info(f"marker poses: {pose_array}")

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
