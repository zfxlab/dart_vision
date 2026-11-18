"""Frame pairing and transactional PNG export, independent of ROS and the GUI."""

import csv
import io
import json
import os
import shutil
import threading
import uuid
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np

CAMERAS = ("camera_1", "camera_2")
CSV_FIELDS = ("pair_id", *CAMERAS, "split", "camera_1_rois", "camera_2_rois")


@dataclass(frozen=True)
class Frame:
    image: np.ndarray
    stamp_ns: int
    received_mono_ns: int
    received_ros_ns: int
    received_index: int
    frame_id: str
    source_encoding: str
    saved_encoding: str

    @property
    def mode(self):
        return (self.image.shape, self.source_encoding, self.saved_encoding, self.frame_id)


class PairBuffer:
    def __init__(self, queue_size, max_delta_ns, max_age_ns):
        if queue_size <= 0 or max_delta_ns < 0 or max_age_ns <= 0:
            raise ValueError("Invalid pairing limits")
        self.frames = {camera: deque(maxlen=queue_size) for camera in CAMERAS}
        self.max_delta_ns = max_delta_ns
        self.max_age_ns = max_age_ns
        self.used = {camera: -1 for camera in CAMERAS}

    def add(self, camera, frame):
        self.frames[camera].append(frame)

    def clear(self):
        for queue in self.frames.values():
            queue.clear()
        self.used = dict.fromkeys(CAMERAS, -1)

    def select(self, now_ros_ns, now_mono_ns, since_ros_ns, since_mono_ns):
        candidates = {}
        for camera in CAMERAS:
            candidates[camera] = [
                frame
                for frame in self.frames[camera]
                if frame.stamp_ns >= since_ros_ns
                and frame.received_mono_ns >= since_mono_ns
                and frame.received_index > self.used[camera]
                and 0 <= now_ros_ns - frame.stamp_ns <= self.max_age_ns
                and 0 <= now_mono_ns - frame.received_mono_ns <= self.max_age_ns
            ]
        pairs = [
            (abs(a.stamp_ns - b.stamp_ns), -min(a.stamp_ns, b.stamp_ns), a, b)
            for a in candidates[CAMERAS[0]]
            for b in candidates[CAMERAS[1]]
            if abs(a.stamp_ns - b.stamp_ns) <= self.max_delta_ns
        ]
        if not pairs:
            return None
        _, _, a, b = min(pairs, key=lambda pair: pair[:2])
        return dict(zip(CAMERAS, (a, b), strict=True))

    def consume(self, pair):
        for camera, frame in pair.items():
            self.used[camera] = frame.received_index


def _durable_write(path, content):
    with path.open("xb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


class SessionWriter:
    """Commit the CSV last; incomplete files never become a selectable calibration pair."""

    def __init__(self, output_root, configuration):
        root = Path(output_root).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")
        self.path = root / f"stereo_{timestamp}_{uuid.uuid4().hex[:8]}"
        self.path.mkdir()
        for name in (*CAMERAS, "metadata"):
            (self.path / name).mkdir()
        self.rows = []
        self.next_id = 1
        self.modes = None
        self.lock = threading.Lock()
        configuration = {
            "schema_version": 1,
            "dataset_format": "stereo_calib_pairs_v2",
            "acquisition_mode": "free_run_receive_time_pairing",
            "timestamp_semantics": "source image header; current Hik driver uses host receipt time",
            "configuration": configuration,
        }
        _durable_write(
            self.path / "session.json",
            (
                json.dumps(configuration, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            ).encode(),
        )
        _durable_write(self.path / "pairs.csv", self._csv([]))

    @staticmethod
    def _csv(rows):
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
        return stream.getvalue().encode()

    def save(self, pair, split, metadata):
        if split not in ("train", "validation"):
            raise ValueError("split must be train or validation")
        if set(pair) != set(CAMERAS):
            raise ValueError("Both cameras are required")
        modes = {camera: frame.mode for camera, frame in pair.items()}
        images = [pair[camera].image for camera in CAMERAS]
        if images[0].shape[:2] != images[1].shape[:2]:
            raise ValueError("Camera resolutions differ; stereo_calib requires matching sizes")
        encoded = {}
        for camera, frame in pair.items():
            if frame.image.dtype != np.uint8 or not (
                frame.image.ndim == 2 or (frame.image.ndim == 3 and frame.image.shape[2] == 3)
            ):
                raise ValueError("Only 8-bit mono or three-channel images can be saved")
            ok, png = cv2.imencode(".png", frame.image)
            if not ok:
                raise OSError(f"Could not encode {camera} as PNG")
            encoded[camera] = png.tobytes()
        with self.lock:
            if self.modes is not None and self.modes != modes:
                raise ValueError("Image mode or frame changed; start a new capture session")
            pair_id = f"{self.next_id:06d}"
            self.next_id += 1
            row = {"pair_id": pair_id, "split": split}
            row.update({camera: f"{camera}/{pair_id}.png" for camera in CAMERAS})
            row.update(camera_1_rois="", camera_2_rois="")
            detail = dict(metadata, pair_id=pair_id, split=split)
            detail["pair_delta_ns"] = abs(pair[CAMERAS[0]].stamp_ns - pair[CAMERAS[1]].stamp_ns)
            detail["images"] = {}
            for camera, frame in pair.items():
                detail["images"][camera] = {
                    "path": row[camera],
                    "stamp_ns": frame.stamp_ns,
                    "received_ros_ns": frame.received_ros_ns,
                    "received_mono_ns": frame.received_mono_ns,
                    "received_index": frame.received_index,
                    "frame_id": frame.frame_id,
                    "width": frame.image.shape[1],
                    "height": frame.image.shape[0],
                    "source_encoding": frame.source_encoding,
                    "saved_encoding": frame.saved_encoding,
                }
            payloads = {row[camera]: encoded[camera] for camera in CAMERAS}
            payloads[f"metadata/{pair_id}.json"] = (
                json.dumps(detail, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            ).encode()
            pending = self.path / f".pending_{uuid.uuid4().hex}"
            pending.mkdir()
            published = []
            try:
                for index, (relative, content) in enumerate(payloads.items()):
                    staged = pending / str(index)
                    _durable_write(staged, content)
                    destination = self.path / relative
                    # An atomic hard link publishes without overwriting any existing file.
                    os.link(staged, destination)
                    published.append(destination)
                manifest = pending / "pairs.csv"
                _durable_write(manifest, self._csv([*self.rows, row]))
                os.replace(manifest, self.path / "pairs.csv")
            except BaseException:
                for destination in published:
                    destination.unlink(missing_ok=True)
                raise
            finally:
                shutil.rmtree(pending, ignore_errors=True)
            self.rows.append(row)
            self.modes = modes
            return pair_id
