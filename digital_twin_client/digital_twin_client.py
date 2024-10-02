import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import uuid
from enum import IntEnum, auto
import csv
import os
from datetime import datetime

from digital_twin_client.client import Client
from digital_twin_client.gui_tool import Button, DrawTool


def image_resize(image, width=1280):
    h, w = image.shape[:2]
    height = round(h * (width / w))
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    return image


def save_local(qr, value, img, robot_id, result_folder='save_folder/', logger=print):
    csv_file = os.path.join(result_folder, 'data.csv')
    image_directory = os.path.join(result_folder, 'image')

    # result_folderが存在しない場合は自動作成
    if not os.path.exists(result_folder):
        os.makedirs(result_folder)
        logger(f"{result_folder} を作成しました。")

    # CSVファイルが存在しない場合は作成し、ヘッダーを書き込み
    if not os.path.exists(csv_file):
        with open(csv_file, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'QR', 'Value',
                            'Image Filename', 'Robot ID'])
        logger(f"{csv_file} を作成し、ヘッダーを追加しました。")

    # imageディレクトリが存在しない場合は作成
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)
        logger(f"{image_directory} ディレクトリを作成しました。")

    # タイムスタンプの生成（ISO 8601形式）
    timestamp = datetime.now().isoformat()

    # ディレクトリ内の画像ファイル数をカウントして次のファイル名を決定
    existing_images = [f for f in os.listdir(
        image_directory) if f.endswith('.jpg')]
    row_count = len(existing_images) + 1  # 現在の画像数に1を足して次の番号を決定

    # 画像ファイル名の作成
    image_filename = f'result_{row_count}.jpg'
    image_path = os.path.join(image_directory, image_filename)
    logger(f"{image_directory}に画像ファイル{image_filename}を生成しました。")

    # 画像ファイルの保存
    cv2.imwrite(image_path, img)

    # CSVファイルへの追記モードで書き込み
    with open(csv_file, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            timestamp,
            qr,
            value,
            image_filename,
            robot_id
        ])


class ResultKind(IntEnum):
    NONE = 0
    METER = auto()
    RUST = auto()
    CRACK = auto()
    TEMP = auto()
    BULB = auto()
    ENVIRONMENT = auto()
    VICTIM = auto()
    DEBRIS = auto()


class DigitalTwinClientNode(Node):

    def __init__(self):
        super().__init__('digital_twin_client')

        # 通信クライアントの初期化
        self.declare_parameter('host', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mac_id', '%012x' % uuid.getnode())

        host = self.get_parameter(
            'host').get_parameter_value().string_value
        self.robot_id = self.get_parameter(
            'robot_id').get_parameter_value().string_value
        mac_id = self.get_parameter(
            'mac_id').get_parameter_value().string_value

        try:
            self.client = Client(
                host, self.robot_id, mac_id, self.get_logger().info)
        except Exception as e:
            self.client = None
            self.get_logger().error(str(e))

        # GUIやメイン処理周りの初期化
        self.kind = ResultKind.NONE
        self.send_subscription = self.create_subscription(
            String,
            'send_topic',
            self.send_callback,
            1)

        # 現在の結果のPublish
        self.timer = self.create_timer(0.1, self.publish_current_result)
        self.result_status_publisher = self.create_publisher(
            String, 'current_result_status', 1)
        self.result_value_publisher = self.create_publisher(
            String, 'current_result_value', 1)
        self.result_qr_publisher = self.create_publisher(
            String, 'current_result_qr', 1)

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
        self.crack_value = ""
        self.crack_image_subscription = self.create_subscription(
            Image,
            'crack_result_image',
            self.crack_image_callback,
            10)
        self.crack_value_subscription = self.create_subscription(
            String,
            'crack_result_txt',
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

        self.bulb_image = self.none_image.copy()
        self.bulb_value = ""
        self.bulb_image_subscription = self.create_subscription(
            Image,
            'bulb_param_image',
            self.bulb_image_callback,
            10)
        self.bulb_value_subscription = self.create_subscription(
            String,
            'bulb_param_result',
            self.bulb_value_callback,
            10)

        self.environment_image = self.none_image.copy()
        self.victim_image = self.none_image.copy()
        self.debris_image = self.none_image.copy()
        self.photo_text = ""
        self.environment_image_subscription = self.create_subscription(
            Image,
            'environment_image',
            self.environment_image_callback,
            10)
        self.victim_image_subscription = self.create_subscription(
            Image,
            'victim_image',
            self.victim_image_callback,
            10)
        self.debris_image_subscription = self.create_subscription(
            Image,
            'debris_image',
            self.debris_image_callback,
            10)
        self.photo_text_subscription = self.create_subscription(
            String,
            'photo_txt_result',
            self.photo_text_callback,
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
                             self.crack_value, self.temperature_value, self.bulb_value,
                             self.photo_text, self.photo_text, self.photo_text
                             ]
            result_value = result_values[self.kind]

            result_images = [self.none_image, self.pressure_image, self.rust_image,
                             self.crack_image, self.temperature_image, self.bulb_image,
                             self.environment_image, self.victim_image, self.debris_image,
                             ]
            result_image = result_images[self.kind]

            # 結果が取得できているか
            is_available_sending = True
            if self.kind == ResultKind.NONE:
                is_available_sending = False

            # 送信ボタンが押されたかの判定
            if is_available_sending and send_button.is_clicked(clicked_pos):
                # jpegに変換(サイズ変更が必要かも)
                before_converted_image = image_resize(result_image.copy())
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
                result, encoded_image = cv2.imencode(
                    '.jpg', before_converted_image, encode_param)
                if False == result:
                    self.get_logger().error('could not encode image!')
                    break
                image_sended = encoded_image.tobytes()

                # ローカルへの保存
                save_local(self.qr_value, str(result_value), result_image,
                           self.robot_id, logger=self.get_logger().info)

                # 送信
                if self.client != None:
                    try:
                        self.client.request(self.qr_value,
                                            str(result_value), image_sended)
                    except Exception as e:
                        self.get_logger().error("Faild sending to RMS: "+str(e))
                else:
                    self.get_logger().error("No connection to RMS")

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
            tool = DrawTool(1280, 720, (0, 0, 0))

            # 現在の状態
            status_texts = ["none", "meter", "rust",
                            "crack", "temperature", "bulb",
                            "environment", "victim", "debris",
                            ]
            current_status = status_texts[self.kind]
            tool.draw_at_text(current_status, (Width//2, 55), 2, thickness=2)

            # QRの値とその他の値
            tool.draw_at_text(f"QR: {self.qr_value}",
                              (1*Width//4, 145), thickness=2)
            tool.draw_at_text(
                f"{current_status}: {result_value}", (3*Width//4, 145), thickness=2)

            # QR画像の表示
            target_width = 4*Width//10
            target_height = 4*Height//10

            h, w = self.qr_image.shape[:2]
            size = (w*target_height//h, target_height)
            if h*target_width <= target_height*w:
                size = (target_width, h*target_width//w)
            tool.draw_image(self.qr_image, (40, 175), size)

            # その他の結果の表示
            h, w = result_image.shape[:2]
            size = (w*target_height//h, target_height)
            if h*target_width <= target_height*w:
                size = (target_width, h*target_width//w)
            tool.draw_image(result_image, (Width//2+40, 175), size)

            # 送信ボタン
            tool.draw_button(cancel_button, "Cancel", thickness=cv2.FILLED, text_color=(
                0, 0, 255), color=(255, 255, 255), text_thickness=2)
            tool.draw_button(send_button, "Send", thickness=cv2.FILLED, text_color=(
                0, 0, 255), color=(255, 255, 255), text_thickness=2)

            # 表示とマウスの設定
            cv2.imshow("send", tool.image)
            cv2.setMouseCallback('send', is_click, clicked_pos)
            cv2.waitKey(10)
        cv2.destroyAllWindows()

    # 現在保持している結果のPublish
    def publish_current_result(self):
        status_texts = ["None", "Meter", "Rust",
                        "Crack", "Temperature", "Bulb",
                        "Environment", "Victim", "Debris",
                        ]
        result_values = [-100.0, self.pressure_value, self.rust_value,
                         self.crack_value, self.temperature_value, self.bulb_image,
                         self.photo_text, self.photo_text, self.photo_text]

        status_msg = String()
        value_msg = String()
        qr_msg = String()
        status_msg.data = status_texts[self.kind]
        value_msg.data = str(result_values[self.kind])
        qr_msg.data = self.qr_value

        self.result_status_publisher.publish(status_msg)
        self.result_value_publisher.publish(value_msg)
        self.result_qr_publisher.publish(qr_msg)

    # データの受け取り
    def qr_image_callback(self, msg):
        self.get_logger().info("get QR image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
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
            self.get_logger().error(e)
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
            self.get_logger().error(e)
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
            self.get_logger().error(e)
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
            self.get_logger().error(e)
            return
        self.kind = ResultKind.TEMP
        self.temperature_image = cv_image

    def temperature_value_callback(self, msg):
        self.get_logger().info("get temperature value")
        self.temperature_value = msg.data

    def bulb_image_callback(self, msg):
        self.get_logger().info("get bulb image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return
        self.kind = ResultKind.BULB
        self.bulb_image = cv_image

    def bulb_value_callback(self, msg):
        self.get_logger().info("get temperature value")
        self.bulb_value = msg.data

    def environment_image_callback(self, msg):
        self.get_logger().info("get environment image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return
        self.kind = ResultKind.ENVIRONMENT
        self.environment_image = cv_image

    def victim_image_callback(self, msg):
        self.get_logger().info("get victim image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return
        self.kind = ResultKind.VICTIM
        self.victim_image = cv_image

    def debris_image_callback(self, msg):
        self.get_logger().info("get debris image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(e)
            return
        self.kind = ResultKind.DEBRIS
        self.debris_image = cv_image

    def photo_text_callback(self, msg):
        self.get_logger().info("get photo text")
        self.photo_text = msg.data


def main(args=None):
    rclpy.init(args=args)

    node = DigitalTwinClientNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
