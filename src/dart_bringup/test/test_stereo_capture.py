"""Pairing and file-transaction tests; no cameras or ROS graph required."""

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from stereo_capture_core import CAMERAS, Frame, PairBuffer, SessionWriter


def frame(stamp=100, arrival=100, index=1, image=None, frame_id="optical"):
    if image is None:
        image = np.arange(12 * 16 * 3, dtype=np.uint8).reshape(12, 16, 3)
    return Frame(image, stamp, arrival, stamp, index, frame_id, "bgr8", "bgr8")


class PairingTest(unittest.TestCase):
    def test_nearest_pair_is_selected_instead_of_unrelated_latest_frames(self):
        buffer = PairBuffer(10, 5, 100)
        buffer.add("camera_1", frame(100))
        buffer.add("camera_1", frame(110, index=2))
        buffer.add("camera_2", frame(102))
        buffer.add("camera_2", frame(130, index=2))
        pair = buffer.select(150, 150, 90, 90)
        self.assertEqual([pair[c].stamp_ns for c in CAMERAS], [100, 102])
        buffer.consume(pair)
        self.assertIsNone(buffer.select(150, 150, 90, 90))

    def test_rejects_old_future_pre_request_and_unsynchronized_frames(self):
        cases = [
            (frame(100), frame(120), 150, 150, 90, 90),
            (frame(100), frame(102), 250, 150, 90, 90),
            (frame(100), frame(102), 150, 250, 90, 90),
            (frame(151), frame(152), 150, 150, 90, 90),
            (frame(100), frame(102), 150, 150, 101, 90),
            (frame(100), frame(102), 150, 150, 90, 101),
        ]
        for a, b, ros, mono, since_ros, since_mono in cases:
            with self.subTest(case=(a.stamp_ns, b.stamp_ns, ros, mono, since_ros, since_mono)):
                buffer = PairBuffer(10, 5, 100)
                buffer.add("camera_1", a)
                buffer.add("camera_2", b)
                self.assertIsNone(buffer.select(ros, mono, since_ros, since_mono))

    def test_queue_bounds_and_clock_reset(self):
        buffer = PairBuffer(1, 5, 100)
        buffer.add("camera_1", frame(100))
        buffer.add("camera_1", frame(130, index=2))
        buffer.add("camera_2", frame(102))
        self.assertIsNone(buffer.select(150, 150, 90, 90))
        buffer.clear()
        self.assertTrue(all(not q for q in buffer.frames.values()))


class SessionWriterTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.writer = SessionWriter(temporary.name, {"purpose": "test"})
        self.pair = {"camera_1": frame(frame_id="one"), "camera_2": frame(102, frame_id="two")}

    def rows(self):
        with (self.writer.path / "pairs.csv").open(newline="") as stream:
            return list(csv.DictReader(stream))

    def test_lossless_originals_metadata_split_and_numbering(self):
        before = self.pair["camera_1"].image.copy()
        first = self.writer.save(self.pair, "train", {"request_ros_ns": 90})
        second = self.writer.save(self.pair, "validation", {})
        self.assertEqual((first, second), ("000001", "000002"))
        rows = self.rows()
        self.assertEqual([r["split"] for r in rows], ["train", "validation"])
        for camera in CAMERAS:
            saved = cv2.imread(str(self.writer.path / rows[0][camera]), cv2.IMREAD_UNCHANGED)
            np.testing.assert_array_equal(saved, self.pair[camera].image)
        np.testing.assert_array_equal(self.pair["camera_1"].image, before)
        metadata = json.loads((self.writer.path / "metadata/000001.json").read_text())
        self.assertEqual(metadata["pair_delta_ns"], 2)
        self.assertEqual(metadata["images"]["camera_1"]["frame_id"], "one")
        self.assertEqual(metadata["images"]["camera_1"]["width"], 16)
        self.assertEqual(metadata["request_ros_ns"], 90)

    def test_second_image_failure_rolls_back_the_pair(self):
        import os

        original = os.link
        count = 0

        def fail_second(*args):
            nonlocal count
            count += 1
            if count == 2:
                raise OSError("disk full")
            return original(*args)

        with patch("stereo_capture_core.os.link", side_effect=fail_second):
            with self.assertRaisesRegex(OSError, "disk full"):
                self.writer.save(self.pair, "train", {})
        self.assertEqual(self.rows(), [])
        for name in (*CAMERAS, "metadata"):
            self.assertEqual(list((self.writer.path / name).iterdir()), [])

    def test_manifest_failure_preserves_previously_committed_pairs(self):
        self.writer.save(self.pair, "train", {})
        with patch("stereo_capture_core.os.replace", side_effect=OSError("manifest failure")):
            with self.assertRaises(OSError):
                self.writer.save(self.pair, "validation", {})
        self.assertEqual([row["pair_id"] for row in self.rows()], ["000001"])
        for name in CAMERAS:
            self.assertEqual([p.name for p in (self.writer.path / name).iterdir()], ["000001.png"])

    def test_existing_files_are_never_overwritten(self):
        destination = self.writer.path / "camera_2/000001.png"
        destination.write_bytes(b"existing data")
        with self.assertRaises(FileExistsError):
            self.writer.save(self.pair, "train", {})
        self.assertEqual(destination.read_bytes(), b"existing data")
        self.assertEqual(self.rows(), [])
        self.assertFalse((self.writer.path / "camera_1/000001.png").exists())

    def test_mode_change_or_different_resolutions_are_rejected(self):
        self.writer.save(self.pair, "train", {})
        changed = {camera: frame(image=np.zeros((24, 32, 3), np.uint8)) for camera in CAMERAS}
        with self.assertRaisesRegex(ValueError, "mode"):
            self.writer.save(changed, "train", {})
        changed["camera_2"] = self.pair["camera_2"]
        with self.assertRaisesRegex(ValueError, "resolutions"):
            self.writer.save(changed, "train", {})
        self.assertEqual(len(self.rows()), 1)

    def test_mono_png_and_incomplete_pairs(self):
        mono = np.arange(192, dtype=np.uint8).reshape(12, 16)
        pair = {camera: frame(image=mono) for camera in CAMERAS}
        self.writer.save(pair, "train", {})
        saved = cv2.imread(str(self.writer.path / "camera_1/000001.png"), cv2.IMREAD_UNCHANGED)
        np.testing.assert_array_equal(saved, mono)
        with self.assertRaisesRegex(ValueError, "Both cameras"):
            self.writer.save({"camera_1": frame()}, "train", {})


if __name__ == "__main__":
    unittest.main()
