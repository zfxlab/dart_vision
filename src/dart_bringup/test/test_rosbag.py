"""Offline regression checks; run after sourcing ROS and the built workspace."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import record_bag
from play_replay_bag import parse_range, write_segment
from record_bag import recover_interrupted_sessions, resolve_topics, write_manifest


class RosbagRegressionTest(unittest.TestCase):
    def test_bootstrap_precedes_new_boundary_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, output = Path(temporary) / "source", Path(temporary) / "output"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(source), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=0,
                    name="/tf_static",
                    type="tf2_msgs/msg/TFMessage",
                    serialization_format="cdr",
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=1, name="/event", type="std_msgs/msg/String", serialization_format="cdr"
                )
            )
            for stamp, x in ((100, 1.0), (101, 2.0)):
                transform = TransformStamped()
                transform.header.frame_id = "world"
                transform.child_frame_id = "camera"
                transform.header.stamp.sec = stamp
                transform.transform.translation.x = x
                transform.transform.rotation.w = 1.0
                writer.write(
                    "/tf_static",
                    serialize_message(TFMessage(transforms=[transform])),
                    stamp * 10**9,
                )
            writer.write("/event", serialize_message(String(data="end")), 102 * 10**9)
            writer.close()
            write_segment(source, output, None, 1.0, 2.0)
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(output)), rosbag2_py.ConverterOptions("", "")
            )
            states = []
            while reader.has_next():
                topic, data, stamp = reader.read_next()
                if topic == "/tf_static":
                    states.append(
                        (
                            stamp,
                            deserialize_message(data, TFMessage)
                            .transforms[0]
                            .transform.translation.x,
                        )
                    )
            reader.close()
            self.assertEqual(states, [(101 * 10**9 - 1, 1.0), (101 * 10**9, 2.0)])

    def test_failed_legacy_session_is_recovered_but_closed_failed_task_is_not(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            legacy, closed = root / "dart_vision_old", root / "dart_vision_closed"
            legacy.mkdir()
            closed.mkdir()
            write_manifest(legacy / "manifest.yaml", {"status": "failed"})
            write_manifest(
                closed / "manifest.yaml", {"status": "failed", "storage_status": "complete"}
            )
            with patch("record_bag.recover_session") as recover:
                recover_interrupted_sessions(root, "mcap")
            recover.assert_called_once_with(legacy, "mcap")
            self.assertTrue((legacy / ".recording").exists())

    def test_profile_inheritance_and_range_validation(self):
        profiles = {
            "base": {"topics": ["/tf"]},
            "inputs": {"include": "base", "topics": ["/image", "/tf"]},
        }
        self.assertEqual(resolve_topics(profiles, "inputs"), ["/tf", "/image"])
        for start, end in ((0, 0), (float("nan"), -1), (2, 1), (0, 10)):
            with self.assertRaises(ValueError):
                parse_range(start, end, 3 * 10**9)

    def test_recorder_failure_keeps_recovery_marker(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / "recording.yaml"
            write_manifest(
                config,
                {
                    "recording": {
                        "output": {"root": str(root / "sessions")},
                        "profiles": {"full": {"topics": ["/test"]}},
                    }
                },
            )
            child = MagicMock(returncode=23)
            child.poll.return_value = 23
            snapshot = MagicMock()
            snapshot.thread.ident = None
            with (
                patch.object(sys, "argv", ["record_bag.py", "--config", str(config)]),
                patch.dict("os.environ", {}, clear=True),
                patch("record_bag.subprocess.Popen", return_value=child),
                patch("record_bag.git_state", return_value={}),
                patch("record_bag.RuntimeSnapshot", return_value=snapshot),
                patch("record_bag.signal.signal"),
            ):
                self.assertEqual(record_bag.main(), 23)
            sessions = list((root / "sessions").glob("dart_vision_*"))
            self.assertEqual(len(sessions), 1)
            self.assertTrue((sessions[0] / ".recording").exists())
            self.assertEqual(
                record_bag.read_manifest(sessions[0] / "manifest.yaml")["status"], "failed"
            )


if __name__ == "__main__":
    unittest.main()
