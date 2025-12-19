from sensors.sensor import Sensor
from sensors.camera import Camera
from flask_socketio import SocketIO

import cv2
import base64
import threading
import time
import math
from collections import defaultdict, deque
from typing import Dict, List, Tuple


class ThumbStream:
    def __init__(
        self,
        socketio: SocketIO,
        preview_interval: float = 0.5,
        max_age: float = 10.0,
        sync_tolerance: float = 0.2,
    ):
        """Buffer thumbnails by sensor timestamp and emit synced previews.

        Args:
            socketio: Flask-SocketIO instance to emit on.
            preview_interval: seconds between preview emits (alignment granularity).
            max_age: maximum allowed age (seconds) between candidate sample and emit time.
        """
        self.socketio = socketio
        self.preview_interval = preview_interval
        self.max_age = max_age
    # maximum allowed skew among samples emitted together (seconds)
        self.sync_tolerance = sync_tolerance

        # buffer: sensor_name -> deque of (bucket, timestamp, encoded_thumb, frame_id)
        self._buffer: Dict[str, deque] = defaultdict(deque)
        self._lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._flush_loop, daemon=True)
        self._last_emit_time = None
        self._thread.start()

    def stop(self) -> None:
        """Stop the background flush thread and flush remaining buffers once."""
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        # final flush
        try:
            self._flush_at(math.floor(time.time() / self.preview_interval) * self.preview_interval)
        except Exception:
            pass

    def yield_thumbnails(self, timestamp: float, data, sensor: Sensor, frame_id: str):
        """Receive a thumbnail (with sensor-supplied timestamp) and buffer it for aligned emits.

        Instead of emitting immediately, we store the encoded image and pick appropriate
        samples for each aligned emit so previews across sensors can be synchronized.
        """
        thumb = sensor.post_process_thumbnail(data, frame_id)
        # keep a debug write on disk; harmless and useful during development
        try:
            cv2.imwrite(f"thumb_{sensor.name}_{frame_id}.jpg", thumb)
        except Exception:
            pass

        topic_id = f"/thumb/{sensor.name}/{frame_id}"
        _, buf = cv2.imencode(".jpg", thumb)
        encoded_thumb = base64.b64encode(buf).decode("utf-8")

        # Bucket timestamp to nearest preview interval step so samples near each other align.
        bucket = round(timestamp / self.preview_interval) * self.preview_interval

        with self._lock:
            # append new sample for this sensor
            self._buffer[sensor.name].append((bucket, timestamp, encoded_thumb, frame_id))

            # prune very old entries to avoid memory growth
            now = time.time()
            cutoff = now - (self.max_age * 2)
            for sname, dq in list(self._buffer.items()):
                while dq and dq[0][1] < cutoff:
                    dq.popleft()

    # --- background flush logic -------------------------------------------------
    def _flush_loop(self):
        while not self._stop_event.is_set():
            # sleep until next aligned boundary
            now = time.time()
            sleep_for = self.preview_interval - (now % self.preview_interval)
            # avoid tiny sleeps
            if sleep_for <= 0:
                sleep_for = self.preview_interval
            self._stop_event.wait(sleep_for)
            if self._stop_event.is_set():
                break

            emit_time = math.floor(time.time() / self.preview_interval) * self.preview_interval
            # avoid duplicate emits for same boundary
            if self._last_emit_time is not None and emit_time <= self._last_emit_time:
                continue
            try:
                self._flush_at(emit_time)
            except Exception:
                # protect background thread
                pass
            self._last_emit_time = emit_time

    def _flush_at(self, emit_time: float):
        """Pick one sample per sensor closest to emit_time (within max_age) and emit them."""
        # candidates: sensor_name -> (best_idx, bucket, timestamp, encoded, fid)
        candidates = {}

        now = time.time()

        with self._lock:
            # gather best candidate per sensor (closest to emit_time within max_age)
            for sensor_name, dq in list(self._buffer.items()):
                if not dq:
                    continue

                best_idx = None
                best_dist = None
                for idx, (bucket, timestamp, encoded, fid) in enumerate(dq):
                    dist = abs(bucket - emit_time)
                    if dist <= self.max_age:
                        if best_dist is None or dist < best_dist:
                            best_dist = dist
                            best_idx = idx

                if best_idx is not None:
                    bucket, timestamp, encoded, fid = dq[best_idx]
                    candidates[sensor_name] = (best_idx, bucket, timestamp, encoded, fid)

        if not candidates:
            return

        # Build list sorted by timestamp to find the largest synchronized cluster
        cand_list = []
        for sname, (idx, bucket, timestamp, encoded, fid) in candidates.items():
            cand_list.append((sname, idx, bucket, timestamp, encoded, fid))

        cand_list.sort(key=lambda x: x[3])  # sort by timestamp

        # sliding window to find largest cluster with max-min <= sync_tolerance
        best_cluster = (0, 0)  # (start_idx, end_idx inclusive)
        best_size = 0
        n = len(cand_list)
        start = 0
        for end in range(n):
            while start <= end and (cand_list[end][3] - cand_list[start][3]) > self.sync_tolerance:
                start += 1
            size = end - start + 1
            if size > best_size:
                best_size = size
                best_cluster = (start, end)

        if best_size == 0:
            return

        start_idx, end_idx = best_cluster
        to_emit = cand_list[start_idx : end_idx + 1]

        # Now remove emitted samples from buffers (only for those sensors) under lock
        with self._lock:
            for sname, idx, bucket, timestamp, encoded, fid in to_emit:
                dq = self._buffer.get(sname)
                if not dq:
                    continue
                # pop up to and including chosen index
                for _ in range(min(idx + 1, len(dq))):
                    dq.popleft()
                if not dq:
                    del self._buffer[sname]

        # emit outside lock
        for sensor_name, idx, bucket, timestamp, encoded_thumb, fid in to_emit:
            topic_id = f"/thumb/{sensor_name}/{fid}"
            try:
                self.socketio.emit(topic_id, encoded_thumb)
            except Exception:
                pass

