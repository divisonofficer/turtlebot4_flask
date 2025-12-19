import time
from typing import Any


class SQueue:
    class Item:
        timestamp: float
        data: Any

        def __init__(self, timestamp: float, data: Any):
            self.timestamp = timestamp
            self.data = data

    def __init__(self):
        self.queue_dict = {}
        self.max_time_interval = 0.1  # 50ms
        self.max_queue_length = 150
        self.root_key = None
        self.essential_keys = []
        self.none_essential_keys = []

    def register_synchronized_queue(self, key: str, is_root=False, is_essential=False):
        if not key in self.queue_dict:
            self.queue_dict[key] = []
        if is_root:
            self.root_key = key
        if is_essential and not key in self.essential_keys:
            self.essential_keys.append(key)
        elif not is_essential and not key in self.none_essential_keys:
            self.none_essential_keys.append(key)

    def unregister_synchronized_queue(self, key: str):
        if key in self.queue_dict:
            del self.queue_dict[key]
        if key in self.essential_keys:
            self.essential_keys.remove(key)

    def enqueue(self, key: str, timestamp: float, data: Any):
        if not key in self.queue_dict:
            return
        self.queue_dict[key].append(self.Item(timestamp, data))
        if len(self.queue_dict[key]) > self.max_queue_length:
            self.queue_dict[key].pop(0)

    def loop_yield_synchronized_items(self, callback):
        if not self.root_key:
            return
        while True:
            if len(self.queue_dict[self.root_key]) == 0:
                continue
            root_item = self.queue_dict[self.root_key][0]
            ret_items = {}
            ret_items[self.root_key] = root_item

            # 5초 이상 된 프레임부터 분석
            if time.time() - root_item.timestamp < 5.0:
                time.sleep(0.1)
                continue

            # 매칭된 아이템들의 인덱스를 저장
            matched_indices = {}

            # 모든 다른 큐에 대해 매칭 프레임 찾기
            for key in self.queue_dict:
                if key == self.root_key:
                    continue
                if len(self.queue_dict[key]) == 0:
                    continue

                # 가장 가까운 시간의 프레임 찾기
                best_match = None
                best_time_diff = float("inf")
                best_index = -1

                for i, item in enumerate(self.queue_dict[key]):
                    time_diff = abs(item.timestamp - root_item.timestamp)
                    if (
                        time_diff < self.max_time_interval
                        and time_diff < best_time_diff
                    ):
                        best_match = item
                        best_time_diff = time_diff
                        best_index = i

                # 매칭되는 프레임이 있으면 추가 (인덱스도 저장)
                if best_match is not None:
                    ret_items[key] = best_match
                    matched_indices[key] = best_index

            # Check if all essential keys are present
            all_essential_present = all(key in ret_items for key in self.essential_keys)

            # Count non-essential keys that are present
            non_essential_keys = [
                key for key in self.queue_dict if key in self.none_essential_keys
            ]
            non_essential_present = sum(
                1 for key in non_essential_keys if key in ret_items
            )
            non_essential_threshold = len(self.none_essential_keys) * 0.5
            print(
                f"Synchronized Queue Status: Essential Present: {all_essential_present}, Non-Essential Present: {non_essential_present}/{len(self.none_essential_keys)} (Threshold: {non_essential_threshold})"
            )
            if (
                all_essential_present
                and non_essential_present >= non_essential_threshold
            ):
                #debug : root 대비 가장 빠른 프레임, 가장 느린 프레임간의 시간 차이를 출력
                min_time = root_item.timestamp
                max_time = root_item.timestamp
                for key, item in ret_items.items():
                    if item.timestamp < min_time:
                        min_time = item.timestamp
                    if item.timestamp > max_time:
                        max_time = item.timestamp
                    #print(f"{item.timestamp - root_item.timestamp:.4f} seconds:  {key} Diffs ")
                #time_diff = max_time - min_time
                #print(f"Synchronized Items Time Diff: {time_diff:.4f} seconds")
                
                # callback 호출
                
                
                callback(ret_items)

                # 성공적으로 callback이 호출되었으므로 매칭된 아이템들을 모두 pop
                # 인덱스가 높은 것부터 pop해야 인덱스가 틀어지지 않음
                for key, index in sorted(
                    matched_indices.items(), key=lambda x: x[1], reverse=True
                ):
                    if index < len(self.queue_dict[key]):
                        self.queue_dict[key].pop(index)

            # root 프레임 제거
            self.queue_dict[self.root_key].pop(0)
            time.sleep(0.05)
