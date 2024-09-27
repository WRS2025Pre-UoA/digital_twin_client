import requests
import json

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

    def rust_request(self, id, value, img):
        print(id, value)

    def crack_request(self, id, value, img):
        print(id, value)

    def temperature_request(self, id, value, img):
        print(id, value)