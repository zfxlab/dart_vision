#!/usr/bin/env python3
"""Copy replay inputs to a new bag without deserializing legacy algorithm messages."""

import argparse
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from record_bag import load_configuration, resolve_topics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Original rosbag directory (or session directory)")
    parser.add_argument("output", type=Path, help="New bag directory; must not exist")
    parser.add_argument(
        "--config", type=Path, help="Recording configuration containing the inputs profile"
    )
    args = parser.parse_args()
    source = args.input.expanduser().resolve()
    if (source / "bag" / "metadata.yaml").is_file():
        source /= "bag"
    if not (source / "metadata.yaml").is_file():
        parser.error(f"Bag metadata not found: {source}")
    destination = args.output.expanduser().resolve()
    if destination.exists():
        parser.error(f"Output already exists: {destination}")
    config = args.config or (
        Path(get_package_share_directory("dart_bringup")) / "config" / "rosbag" / "recording.yaml"
    )
    topics = resolve_topics(load_configuration(config)["profiles"], "inputs")
    if (
        not isinstance(topics, list)
        or not topics
        or not all(isinstance(topic, str) and topic.startswith("/") for topic in topics)
    ):
        parser.error("The inputs profile must contain a non-empty list of absolute topics")
    with (source / "metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    available = {
        item["topic_metadata"]["name"]
        for item in metadata["topics_with_message_count"]
        if item["message_count"] > 0
    }
    missing = set(topics) - available
    if missing:
        parser.error(f"Required replay inputs missing or empty: {', '.join(sorted(missing))}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    options = {"output_bags": [{"uri": str(destination), "storage_id": "mcap", "topics": topics}]}
    with tempfile.TemporaryDirectory(prefix="dart_replay_") as temporary:
        options_path = Path(temporary) / "convert.yaml"
        with options_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(options, stream)
        subprocess.run(
            ["ros2", "bag", "convert", "-i", str(source), "-o", str(options_path)],
            check=True,
        )
    print(f"Replay bag: {destination}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
