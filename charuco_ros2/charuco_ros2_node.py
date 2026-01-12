#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import quaternion
import cv2
from cv2 import aruco
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Transform, Point
import message_filters
from tf2_ros import TransformBroadcaster


class CharucoDetector(Node):
    def __init__(self):
        super().__init__("charuco_detector")

        # --- Board configuration ---
        self.dictionary_id = aruco.DICT_5X5_1000
        self.charuco_dict = aruco.getPredefinedDictionary(self.dictionary_id)
        nb_squares_x, nb_squares_y = 12, 9
        square_length = 0.030
        marker_length = 0.022
        self.board = cv2.aruco.CharucoBoard((nb_squares_x, nb_squares_y), square_length, marker_length, self.charuco_dict)

        self.bridge = CvBridge()

        # --- Subscribers with sync ---
        img_topic = "/camera/camera/color/image_raw"
        info_topic = "/camera/camera/color/camera_info"
        image_sub = message_filters.Subscriber(self, Image, img_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)
        self.pub_debug = self.create_publisher(Image, "charuco/debug_image", 10)
        self.pub_debug_info = self.create_publisher(CameraInfo, "charuco/camera_info", 10)

        # ApproximateTime or ExactTime depending on your pipeline
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, info_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # TF
        self.charuco_frame_id = "charuco_board"
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("ChArUco detector node initialized.")

    def sync_callback(self, img_msg: Image, info_msg: CameraInfo):
        # Convert ROS → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, self.charuco_dict)

        if ids is not None and len(ids) > 0:
            # Refine and interpolate ChArUco corners
            retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=frame,
                board=self.board
            )

            # Draw detected ArUco markers (optional)
            aruco.drawDetectedMarkers(frame, corners, ids)

            if retval > 0:
                # Draw ChArUco corners
                aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

                # --- Pose Estimation ---
                camera_matrix = np.array(info_msg.k).reshape(3, 3)
                dist_coeffs = np.array(info_msg.d)

                # camera board
                # Allocate output pose vectors before calling (OpenCV 4.10 requirement)
                rvec_c_b = np.zeros((3, 1), dtype=np.float64)
                tvec_c_b = np.zeros((3, 1), dtype=np.float64)

                success = aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self.board,
                    camera_matrix,
                    dist_coeffs,
                    rvec_c_b,
                    tvec_c_b
                )

                if success:
                    R_c_b, _ = cv2.Rodrigues(rvec_c_b)

                    tf_stamped_c_b = TransformStamped()
                    tf_stamped_c_b.header = img_msg.header
                    tf_stamped_c_b.child_frame_id = self.charuco_frame_id
                    tf_stamped_c_b.transform = np_mat_to_transform(tvec_c_b, R_c_b)
                    self.tf_broadcaster.sendTransform(tf_stamped_c_b)

                    # Draw coordinate axes on the board
                    cv2.drawFrameAxes(
                        frame,
                        camera_matrix,
                        dist_coeffs,
                        rvec_c_b,
                        tvec_c_b,
                        length=0.05  # 5 cm axes
                    )
                    self.get_logger().info("ChArUco pose estimated.")
                else:
                    self.get_logger().warn("Pose estimation failed.")
            else:
                self.get_logger().warn("No ChArUco corners detected.")
        else:
            self.get_logger().warn("No ArUco markers detected.")

        # --- Publish debug image with all drawings ---
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = img_msg.header  # keep timestamps aligned
        self.pub_debug.publish(debug_msg)
        self.pub_debug_info.publish(info_msg)

def np_mat_to_transform(t: np.ndarray, R: np.ndarray) -> Transform:
    """
    Convert (t,R) pair to ros2 Transform msg.

    Args:
        t: The 3x3 NumPy array.
        R: The 3 NumPy array.
        header_frame: The header.frame_id for the Transform.
        child_frame: The child_frame_id for the Transform.
        stamp: Optional timestamp for the header.

    Returns:
        A ROS 2 Transform message.
    """
    transform = Transform()

    # Extract translation
    transform.translation.x = float(t[0])
    transform.translation.y = float(t[1])
    transform.translation.z = float(t[2])

    # Extract rotation as quaternion
    q = quaternion.from_rotation_matrix(R)
    transform.rotation.x = q.x
    transform.rotation.y = q.y
    transform.rotation.z = q.z
    transform.rotation.w = q.w

    return transform

def main(args=None):
    rclpy.init(args=args)
    node = CharucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
