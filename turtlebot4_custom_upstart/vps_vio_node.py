import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image

import tf2_ros as tf2

import numpy as np
import transforms3d as t3d

from robot_vps_ros.vps_ros_client import VpsRosClient


vision_to_robotics_tfmat = np.array(
    [[0.0, 0.0, 1.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
# robotics_to_vision_tfmat = np.linalg.inv(vision_to_robotics_tfmat)
robotics_to_vision_tfmat = np.array(
    [[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)

ENU_ORIGIN = {"lon": 19.07939640241426105, "lat": 47.48623767928393846, "h": 0.0}


class VpsVioNode(Node):
    def __init__(self):
        super().__init__("vps_vio_node")

        self.declare_parameter("vps_server_address", "localhost")
        self.declare_parameter("vps_request_timer_period", 5.0)

        self.declare_parameter("enu_frame", "enu")
        self.declare_parameter("reference_frame", "odom")

        self.declare_parameter("camera_info_topic", "/slam/camera_info")
        self.declare_parameter("image_raw_topic", "/slam/left")
        self.declare_parameter("slam_odom_topic", "/slam/odometry")

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self.camera_info_callback, 10
        )
        self.image_raw_subscription = self.create_subscription(
            Image, self.get_parameter("image_raw_topic").value, self.image_raw_callback, 10
        )
        self.slam_odom_subscription = self.create_subscription(
            PoseStamped, self.get_parameter("slam_odom_topic").value, self.slam_odom_callback, 10
        )

        self.vps_request_timer = self.create_timer(
            self.get_parameter("vps_request_timer_period").value, self.vps_request_timer_callback
        )

        self.camera_info: CameraInfo = None
        self.image_raw: Image = None
        self.slam_odom: PoseStamped = None

        self.vps_ros_client = VpsRosClient(self.get_parameter("vps_server_address").value)

        self.tf_buffer = tf2.Buffer(node=self)
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2.TransformBroadcaster(self)

        self.get_logger().info("VPS VIO Node Started")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def image_raw_callback(self, msg: Image):
        self.image_raw = msg

    def slam_odom_callback(self, msg: PoseStamped):
        self.slam_odom = msg

    def vps_request_timer_callback(self):
        enu_to_end_tfmat = self.get_enu_to_end_tfmat()
        if enu_to_end_tfmat is None:
            return

        slam_world_to_end_tfmat = self.get_slam_world_to_end_tfmat(self.slam_odom)
        if slam_world_to_end_tfmat is None:
            return

        enu_to_slam_world_tfmat = enu_to_end_tfmat @ np.linalg.inv(slam_world_to_end_tfmat)
        trzs = t3d.affines.decompose(enu_to_slam_world_tfmat)
        enu_to_slam_world_tvec = trzs[0]
        enu_to_slam_world_qvec = t3d.quaternions.mat2quat(trzs[1])

        enu_to_slam_world_tf = TransformStamped()
        enu_to_slam_world_tf.header.stamp = self.get_clock().now().to_msg()
        enu_to_slam_world_tf.header.frame_id = "enu"
        enu_to_slam_world_tf.child_frame_id = "slam_world"
        enu_to_slam_world_tf.transform.translation.x = enu_to_slam_world_tvec[0]
        enu_to_slam_world_tf.transform.translation.y = enu_to_slam_world_tvec[1]
        enu_to_slam_world_tf.transform.translation.z = enu_to_slam_world_tvec[2]
        enu_to_slam_world_tf.transform.rotation.w = enu_to_slam_world_qvec[0]
        enu_to_slam_world_tf.transform.rotation.x = enu_to_slam_world_qvec[1]
        enu_to_slam_world_tf.transform.rotation.y = enu_to_slam_world_qvec[2]
        enu_to_slam_world_tf.transform.rotation.z = enu_to_slam_world_qvec[3]

        self.tf_broadcaster.sendTransform(enu_to_slam_world_tf)

    def get_enu_to_end_tfmat(self) -> np.ndarray | None:
        if (self.camera_info is None) or (self.image_raw is None) or (self.slam_odom is None):
            self.get_logger().warn("Camera info, image raw, or SLAM odometry is not available.")
            return None

        # enu_origin_response = self.vps_ros_client.request_enu_origin(
        #     "camera", self.image_raw, self.camera_info
        # )

        # if enu_origin_response is None:
        #     self.get_logger().warn("VPS ENU origin / geopose not found.")
        #     return None

        # if enu_origin_response.enuOrigin != self.vps_ros_client.enu_origin:
        #     self.get_logger().info(f"ENU origin updated to {enu_origin_response.enuOrigin}")
        #     self.vps_ros_client.enu_origin = enu_origin_response.enuOrigin

        # geopose = enu_origin_response.geopose

        geopose_response = self.vps_ros_client.request_geopose("camera", self.camera_info, self.image_raw)
        if geopose_response is None:
            self.get_logger().warn("VPS geopose not found.")
            return None
        
        geopose = geopose_response.geopose
        enu_position = self.vps_ros_client.transform_geo_to_enu(
            geopose.position.lon, geopose.position.lat, geopose.position.h
        )

        enu_to_end_tvec = [enu_position[0], enu_position[1], enu_position[2]]
        enu_to_end_qvec = [geopose.quaternion.w, geopose.quaternion.x, geopose.quaternion.y, geopose.quaternion.z]
        enu_to_end_rmat = t3d.quaternions.quat2mat(enu_to_end_qvec)
        enu_to_end_tfmat = t3d.affines.compose(enu_to_end_tvec, enu_to_end_rmat, np.ones(3))

        return enu_to_end_tfmat

    def get_slam_world_to_end_tfmat(self, slam_odom: PoseStamped) -> np.ndarray:
        if self.slam_odom is None:
            self.get_logger().warn("SLAM odometry not available.")
            return None

        slam_world_to_end_tvec = [
            slam_odom.pose.position.x,
            slam_odom.pose.position.y,
            slam_odom.pose.position.z,
        ]
        slam_world_to_end_qvec = [
            slam_odom.pose.orientation.w,
            slam_odom.pose.orientation.x,
            slam_odom.pose.orientation.y,
            slam_odom.pose.orientation.z,
        ]
        slam_world_to_end_rmat = t3d.quaternions.quat2mat(slam_world_to_end_qvec)
        slam_world_to_end_tfmat = t3d.affines.compose(slam_world_to_end_tvec, slam_world_to_end_rmat, np.ones(3))

        return slam_world_to_end_tfmat


def main(args=None):
    rclpy.init(args=args)
    node = VpsVioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
