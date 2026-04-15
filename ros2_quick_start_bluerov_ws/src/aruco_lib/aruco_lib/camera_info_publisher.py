#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')

        # paramètres
        self.declare_parameter('yaml_file', '')
        self.declare_parameter('frame_id', 'camera')

        yaml_file = "/home/tbelier/Documents/GIT/BlueROVEX/ros2_quick_start_bluerov_ws/src/aruco_lib/aruco_lib/castorCalibration.yaml"#self.get_parameter('yaml_file').value
        self.frame_id = self.get_parameter('frame_id').value

        # publisher
        self.publisher = self.create_publisher(
            CameraInfo,
            '/image/camera_info',
            10
        )

        # lecture YAML
        with open(yaml_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        self.camera_info = CameraInfo()

        self.camera_info.width = calib_data["image_width"]
        self.camera_info.height = calib_data["image_height"]

        self.camera_info.k = calib_data["camera_matrix"]["data"]
        self.camera_info.d = calib_data["distortion_coefficients"]["data"]
        self.camera_info.r = calib_data["rectification_matrix"]["data"]
        self.camera_info.p = calib_data["projection_matrix"]["data"]

        self.camera_info.distortion_model = calib_data["distortion_model"]

        self.timer = self.create_timer(0.1, self.publish_info)

    def publish_info(self):

        msg = self.camera_info
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = CameraInfoPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()