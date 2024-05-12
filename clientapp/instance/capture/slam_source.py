import requests


class SlamSource:
    SLAM_URL = "http://localhost/slam/"

    def __init__(self):
        pass

    def search_map_name_empty(self, map_name: str):
        response = requests.get(f"{self.SLAM_URL}map/{map_name}")
        if response.status_code == 200:
            return False
        return True

    def request_map_save(self, map_name: str):
        """
        return 0 if success
        return message if failed
        """
        response = requests.post(
            f"{self.SLAM_URL}map/save", json={"filename": map_name}
        )
        if response.status_code == 200:
            return None
        return response

    def request_map_save_and_clean(self, map_name: str):
        with self.request_map_save(map_name) as response:
            if response:
                return response

        response = requests.get(f"{self.SLAM_URL}cancel")
        if response.status_code == 200:
            return None
        return response.status_code

    def request_map_load(self, map_name: str):
        """
        return 0 if success
        return message if failed
        """
        response = requests.post(
            f"{self.SLAM_URL}map/load", json={"filename": map_name}
        )
        if response.status_code == 200:
            return None
        return response.content
