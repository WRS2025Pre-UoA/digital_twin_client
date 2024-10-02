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

    def request(self, id, value, img, logger=print):
        logger(id, value)

        url = 'http://'+self.host+'/rms_wrs/api/set_eqpt_val.php'
        values = {
            "eqpt_nm": id,
            "value": value,
            "mac_id": self.mac_id
        }
        files = {'image': ("meter_result.jpg", img)}

        response = requests.post(url, files=files, data=values)
        if response.status_code != requests.codes.ok:
            raise Exception(
                f"Request Status Error: {response.status_code}")
        if json.loads(response.text)['status'] == 'success':
            logger('post success')
        else:
            raise Exception(f"Request Error: {response.text}")
