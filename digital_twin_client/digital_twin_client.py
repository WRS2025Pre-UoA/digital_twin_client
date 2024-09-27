from re import A
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64

import cv2
from cv_bridge import CvBridge, CvBridgeError

import os
import requests
import json
import uuid


class Client:
    def __init__(self, host, robot_id, mac_id, logger=print):
        self.host = host
        self.mac_id = mac_id

        values = {"rob_id": robot_id, "mac_id": mac_id}
        url = 'http://'+host+'/rms_wrs/api/set_mac_id.php'  # to register the robot

        self.session = requests.Session()  # classが終了したらsessionが終わるようにメソッド変数として持たせる
        response = self.session.post(url, json=values)
        if response.status_code != requests.codes.ok:
            raise Exception(
                f"Request Status Error: {response.status_code}")

        if response.json()['status'] == 'success':
            logger("Start Session")
            logger(json.dumps(response.json(), indent=4))
        else:
            raise Exception(f"Request Error: {response.json()}")

    def meter_request(self, id, value, img):
        print(id, value)
        cv2.imshow(img)
        cv2.waitKey(0)

    def rust_request(self, id, value, img):
        print(id, value)
        cv2.imshow(img)
        cv2.waitKey(0)

    def crack_request(self, id, value, img):
        print(id, value)
        cv2.imshow(img)
        cv2.waitKey(0)

    def temperature_request(self, id, value, img):
        print(id, value)
        cv2.imshow(img)
        cv2.waitKey(0)


class DigitalTwinClientNode(Node):
    def __init__(self):
        super().__init__('digital_twin_client')

        self.declare_parameter('host', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mac_id', '%012x' % uuid.getnode())

        self.host = self.get_parameter(
            'host').get_parameter_value().string_value
        self.robot_id = self.get_parameter(
            'robot_id').get_parameter_value().string_value
        self.mac_id = self.get_parameter(
            'mac_id').get_parameter_value().string_value

        self.send_subscription = self.create_subscription(
            String,
            'send_topic',
            self.send_callback,
            10)

        self.qr_image = None
        self.qr_value = ""
        self.qr_image_subscription = self.create_subscription(
            Image,
            'qr_result_image',
            self.qr_image_callback,
            10)
        self.qr_value_subscription = self.create_subscription(
            String,
            'qr_result_value',
            self.qr_value_callback,
            10)

        self.pressure_image = None
        self.pressure_value = ""
        self.pressure_image_subscription = self.create_subscription(
            Image,
            'pressure_result_image',
            self.pressure_image_callback,
            10)
        self.pressure_value_subscription = self.create_subscription(
            Float64,
            'pressure_result_value',
            self.pressure_value_callback,
            10)

        self.rust_image = None
        self.rust_value = ""
        self.rust_image_subscription = self.create_subscription(
            Image,
            'rust_result_image',
            self.rust_image_callback,
            10)
        self.rust_value_subscription = self.create_subscription(
            Float64,
            'rust_result_value',
            self.rust_value_callback,
            10)

        self.crack_image = None
        self.crack_value = ""
        self.crack_image_subscription = self.create_subscription(
            Image,
            'crack_result_image',
            self.crack_image_callback,
            10)
        self.crack_value_subscription = self.create_subscription(
            Float64,
            'crack_result_value',
            self.crack_value_callback,
            10)

        self.temperature_image = None
        self.temperature_value = ""
        self.temperature_image_subscription = self.create_subscription(
            Image,
            'temperature_result_image',
            self.temperature_image_callback,
            10)
        self.temperature_value_subscription = self.create_subscription(
            Float64,
            'temperature_result_value',
            self.temperature_value_callback,
            10)

    def send_callback(self, msg):
        self.get_logger().info("start sending to rms")

    def qr_image_callback(self, msg):
        self.qr_image = msg

    def qr_value_callback(self, msg):
        self.qr_value = msg

    def pressure_image_callback(self, msg):
        self.pressure_image = msg

    def pressure_value_callback(self, msg):
        self.pressure_value = msg

    def rust_image_callback(self, msg):
        self.rust_image = msg

    def rust_value_callback(self, msg):
        self.rust_value = msg

    def crack_image_callback(self, msg):
        self.crack_image = msg

    def crack_value_callback(self, msg):
        self.crack_value = msg

    def temperature_image_callback(self, msg):
        self.temperature_image = msg

    def temperature_value_callback(self, msg):
        self.temperature_value = msg


def main(args=None):
    rclpy.init(args=args)

    node = DigitalTwinClientNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
