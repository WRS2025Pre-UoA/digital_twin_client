import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import uuid
from enum import IntEnum, auto

from digital_twin_client.client import Client
from digital_twin_client.gui_tool import Button, DrawTool


class ResultKind(IntEnum):
    NONE = 0
    METER = auto()
    RUST = auto()
    CRACK = auto()
    TEMP = auto()


class DigitalTwinClientNode(Node):

    def __init__(self):
        super().__init__('digital_twin_client')

        # 通信クライアントの初期化
        self.declare_parameter('host', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mac_id', '%012x' % uuid.getnode())

        host = self.get_parameter(
            'host').get_parameter_value().string_value
        robot_id = self.get_parameter(
            'robot_id').get_parameter_value().string_value
        mac_id = self.get_parameter(
            'mac_id').get_parameter_value().string_value

        self.client = Client(
            host, robot_id, mac_id, self.get_logger().info)

        # GUIやメイン処理周りの初期化
        self.kind = ResultKind.NONE
        self.send_subscription = self.create_subscription(
            String,
            'send_topic',
            self.send_callback,
            1)

        # 結果データの受け取りの初期化
        self.bridge = CvBridge()

        self.none_image = np.full((320, 640, 3), (255, 255, 0), dtype=np.uint8)

        self.qr_image = self.none_image.copy()
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

        self.pressure_image = self.none_image.copy()
        self.pressure_value = 0.0
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

        self.rust_image = self.none_image.copy()
        self.rust_value = 0.0
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

        self.crack_image = self.none_image.copy()
        self.crack_value = 0.0
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

        self.temperature_image = self.none_image.copy()
        self.temperature_value = 0.0
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

    # GUIやメイン処理周り
    def send_callback(self, msg):
        self.get_logger().info("start sending to rms")
        Width = 1280
        Height = 720

        def is_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                param[0] = x
                param[1] = y
            else:
                param[0] = -1
                param[1] = -1
        clicked_pos = [-1, -1]
        send_button = Button((Width-250, Height-120), (200, 100))
        cancel_button = Button((50, Height-120), (200, 100))

        while True:
            # 各結果の配列
            result_values = ["", self.pressure_value, self.rust_value,
                             self.crack_value, self.temperature_value]
            result_value = result_values[self.kind]

            result_images = [self.none_image, self.pressure_image,
                             self.rust_image, self.crack_image, self.temperature_image]
            result_image = result_images[self.kind]

            # 結果が取得できているか
            is_available_sending = True
            if self.kind == ResultKind.NONE:
                is_available_sending = False

            # 送信ボタンが押されたかの判定
            if is_available_sending and send_button.is_clicked(clicked_pos):
                send_funcs = [self.client.meter_request, self.client.rust_request,
                              self.client.crack_request, self.client.temperature_request]

                # jpegに変換(サイズ変更が必要かも)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
                result, encoded_image = cv2.imencode(
                    '.jpg', result_image, encode_param)
                if False == result:
                    self.get_logger().error('could not encode image!')
                    break
                image_sended = encoded_image.tobytes()

                # 送信
                send_funcs[self.kind-1](self.qr_value,
                                        result_value, image_sended)

                # 初期化
                self.kind = ResultKind.NONE

                self.qr_image = self.none_image.copy()
                self.result_image = self.none_image.copy()

                self.qr_value = ""
                self.result_value = 0.0
                break
            # キャンセルボタンが押されたかの判定
            if cancel_button.is_clicked(clicked_pos):
                break

            # 描画
            tool = DrawTool(1280, 720, (127, 127, 127))

            # 現在の状態
            status_texts = ["none", "meter",
                            "rust", "crack", "temperature"]
            current_status = status_texts[self.kind]
            tool.draw_at_text(current_status, (Width//2, 15), 2)

            # QRの値とその他の値
            tool.draw_at_text(f"QR: {self.qr_value}", (1*Width//4, 50), 1)
            tool.draw_at_text(
                f"{current_status}: {result_value}", (3*Width//4, 50), 1)

            # QR画像の表示
            target_width = 4*Width//10
            target_height = 4*Height//10

            h, w = self.qr_image.shape[:2]
            size = (w*target_height//h, target_height)
            if h*target_width <= target_height*w:
                size = (target_width, h*target_width//w)
            tool.draw_image(self.qr_image, (20, 75), size)

            # その他の結果の表示
            h, w = result_image.shape[:2]
            size = (w*target_height//h, target_height)
            if h*target_width <= target_height*w:
                size = (target_width, h*target_width//w)
            tool.draw_image(result_image, (Width//2+20, 75), size)

            # 送信ボタン
            tool.draw_button(cancel_button, "Cancel", thickness=2)
            tool.draw_button(send_button, "Send", thickness=2)

            # 表示とマウスの設定
            cv2.imshow("send", tool.image)
            cv2.setMouseCallback('send', is_click, clicked_pos)
            cv2.waitKey(10)
        cv2.destroyAllWindows()

    # データの受け取り
    def qr_image_callback(self, msg):
        self.get_logger().info("get QR image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.qr_image = cv_image

    def qr_value_callback(self, msg):
        self.get_logger().info("get QR value")
        self.qr_value = msg.data

    def pressure_image_callback(self, msg):
        self.get_logger().info("get meter image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.kind = ResultKind.METER
        self.pressure_image = cv_image

    def pressure_value_callback(self, msg):
        self.get_logger().info("get meter value")
        self.pressure_value = msg.data

    def rust_image_callback(self, msg):
        self.get_logger().info("get rust image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.kind = ResultKind.RUST
        self.rust_image = cv_image

    def rust_value_callback(self, msg):
        self.get_logger().info("get rust value")
        self.rust_value = msg.data

    def crack_image_callback(self, msg):
        self.get_logger().info("get crack image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.kind = ResultKind.CRACK
        self.crack_image = cv_image

    def crack_value_callback(self, msg):
        self.get_logger().info("get crack value")
        self.crack_value = msg.data

    def temperature_image_callback(self, msg):
        self.get_logger().info("get temperature image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            print(e)
            return
        self.kind = ResultKind.TEMP
        self.temperature_image = cv_image

    def temperature_value_callback(self, msg):
        self.get_logger().info("get temperature value")
        self.temperature_value = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = DigitalTwinClientNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
