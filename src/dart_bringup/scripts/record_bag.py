#!/usr/bin/env python3

import argparse
import fcntl
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import yaml


def positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"recording.storage.{name} must be a positive integer")
    return value


def find_repository(start):
    for candidate in (start, *start.parents):
        if (candidate / ".git").exists():
            return candidate
    return None


def git_state(config_path):
    repository = find_repository(Path.cwd()) or find_repository(config_path.resolve())
    if repository is None:
        return {"commit": "unknown", "dirty": None}
    try:
        commit = subprocess.run(
            ["git", "-C", str(repository), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "-C", str(repository), "status", "--porcelain"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "unknown", "dirty": None}


def load_configuration(path):
    with path.open(encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file) or {}
    recording = document.get("recording")
    if not isinstance(recording, dict):
        raise ValueError("recording.yaml must contain a 'recording' mapping")
    return recording


def resolve_topics(profiles, profile_name, resolving=None):
    if not isinstance(profiles, dict) or profile_name not in profiles:
        raise ValueError(f"unknown recording profile: {profile_name}")
    resolving = set() if resolving is None else resolving
    if profile_name in resolving:
        raise ValueError(f"recording profile include cycle at: {profile_name}")
    resolving.add(profile_name)

    profile = profiles[profile_name]
    if not isinstance(profile, dict):
        raise ValueError(f"recording profile '{profile_name}' must be a mapping")
    topics = []
    included = profile.get("include")
    if included is not None:
        if not isinstance(included, str):
            raise ValueError(f"recording profile '{profile_name}'.include must be a string")
        topics.extend(resolve_topics(profiles, included, resolving))
    own_topics = profile.get("topics", [])
    if not isinstance(own_topics, list) or not all(
        isinstance(topic, str) and topic.startswith("/") for topic in own_topics
    ):
        raise ValueError(f"recording profile '{profile_name}'.topics must contain absolute topics")
    topics.extend(own_topics)
    resolving.remove(profile_name)
    return list(dict.fromkeys(topics))


def expanded_path(value, config_path):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("recording.output.root must be a non-empty path")
    path = Path(os.path.expandvars(os.path.expanduser(value)))
    if path.is_absolute():
        return path.resolve()
    repository = find_repository(config_path.resolve()) or find_repository(Path.cwd())
    if repository is None:
        raise ValueError("a relative recording.output.root requires a Git worktree")
    return (repository / path).resolve()


def copy_configuration_snapshot(config_path, recording, destination):
    config_root = config_path.parent
    destination.mkdir()
    shutil.copy2(config_path, destination / config_path.name)
    snapshot_files = recording.get("snapshot_files", [])
    if not isinstance(snapshot_files, list):
        raise ValueError("recording.snapshot_files must be a list")
    for relative_name in snapshot_files:
        if not isinstance(relative_name, str):
            raise ValueError("recording.snapshot_files must contain relative paths")
        relative_path = Path(relative_name)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ValueError(f"invalid recording snapshot path: {relative_name}")
        source = config_root / relative_path
        if not source.is_file():
            raise FileNotFoundError(f"recording snapshot file not found: {source}")
        target = destination / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def write_manifest(path, manifest):
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", encoding="utf-8") as manifest_file:
        yaml.safe_dump(manifest, manifest_file, sort_keys=False, allow_unicode=True)
    temporary.replace(path)


def read_manifest(path):
    if not path.is_file():
        return {}
    try:
        with path.open(encoding="utf-8") as manifest_file:
            manifest = yaml.safe_load(manifest_file) or {}
        return manifest if isinstance(manifest, dict) else {}
    except (OSError, yaml.YAMLError):
        return {}


def rosbag_command_succeeds(arguments):
    return (
        subprocess.run(
            ["ros2", "bag", *arguments],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        == 0
    )


def quarantine_file(source, lost_found):
    lost_found.mkdir(exist_ok=True)
    destination = lost_found / source.name
    if destination.exists():
        destination = lost_found / f"{source.name}.{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.move(str(source), destination)
    return destination.relative_to(lost_found.parent).as_posix()


def mark_recovery_failed(marker, session_path):
    failed_marker = session_path / ".recovery_failed"
    if failed_marker.exists():
        failed_marker = session_path / f".recovery_failed_{time.strftime('%Y%m%d_%H%M%S')}"
    marker.rename(failed_marker)


def recover_session(session_path, storage_id):
    marker = session_path / ".recording"
    bag_path = session_path / "bag"
    manifest_path = session_path / "manifest.yaml"
    manifest = read_manifest(manifest_path)
    recovered_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    quarantined = []

    print(f"Recovering interrupted recording: {session_path}", flush=True)
    if bag_path.is_dir() and rosbag_command_succeeds(["info", str(bag_path)]):
        manifest["status"] = "recovered"
        manifest["recovery"] = {"recovered_at": recovered_at, "quarantined_files": []}
        write_manifest(manifest_path, manifest)
        marker.unlink()
        print("Recording metadata was already complete", flush=True)
        return True

    extension = ".mcap" if storage_id == "mcap" else ".db3"
    storage_files = sorted(bag_path.glob(f"*{extension}")) if bag_path.is_dir() else []
    lost_found = session_path / "lost+found"
    for storage_file in storage_files:
        if not rosbag_command_succeeds(["info", str(storage_file)]):
            quarantined.append(quarantine_file(storage_file, lost_found))

    remaining_files = sorted(bag_path.glob(f"*{extension}")) if bag_path.is_dir() else []
    metadata_path = bag_path / "metadata.yaml"
    if metadata_path.exists():
        quarantined.append(quarantine_file(metadata_path, lost_found))

    recovered = bool(remaining_files) and rosbag_command_succeeds(
        ["reindex", "--storage", storage_id, str(bag_path)]
    )
    recovered = recovered and rosbag_command_succeeds(["info", str(bag_path)])
    manifest["status"] = "recovered" if recovered else "recovery_failed"
    manifest["recovery"] = {
        "recovered_at": recovered_at,
        "quarantined_files": quarantined,
    }
    write_manifest(manifest_path, manifest)
    if recovered:
        marker.unlink()
        print(f"Recovered recording; quarantined {len(quarantined)} file(s)", flush=True)
    else:
        mark_recovery_failed(marker, session_path)
        print("Recording recovery failed; preserved files for manual inspection", flush=True)
    return recovered


def recover_interrupted_sessions(output_root, storage_id):
    for session_path in sorted(output_root.glob("dart_vision_*")):
        if session_path.is_dir() and (session_path / ".recording").is_file():
            recover_session(session_path, storage_id)


def build_command(recording, topics, bag_path):
    storage = recording.get("storage", {})
    if not isinstance(storage, dict):
        raise ValueError("recording.storage must be a mapping")
    storage_id = storage.get("storage_id", "mcap")
    if storage_id not in ("mcap", "sqlite3"):
        raise ValueError("recording.storage.storage_id must be 'mcap' or 'sqlite3'")
    cache_size = positive_integer(
        storage.get("max_cache_size_bytes", 67108864), "max_cache_size_bytes"
    )
    bag_size = positive_integer(storage.get("max_bag_size_bytes", 4294967296), "max_bag_size_bytes")
    bag_duration = positive_integer(
        storage.get("max_bag_duration_seconds", 30), "max_bag_duration_seconds"
    )
    compression_mode = storage.get("compression_mode", "none")
    if compression_mode not in ("none", "file", "message"):
        raise ValueError("recording.storage.compression_mode must be none, file or message")

    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        storage_id,
        "--output",
        str(bag_path),
        "--max-cache-size",
        str(cache_size),
        "--max-bag-size",
        str(bag_size),
        "--max-bag-duration",
        str(bag_duration),
        "--disable-keyboard-controls",
    ]
    if compression_mode != "none":
        command.extend(["--compression-mode", compression_mode, "--compression-format", "zstd"])
    command.append("--topics")
    command.extend(topics)
    return command


def unique_session_path(root):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = root / f"dart_vision_{timestamp}"
    if not candidate.exists():
        return candidate
    return root / f"dart_vision_{timestamp}_{os.getpid()}"


def signal_process_group(process, requested_signal):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, requested_signal)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description="Record dart_vision topics with rosbag2")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--profile")
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    recording = load_configuration(config_path)
    profile = args.profile or recording.get("default_profile", "compressed")
    topics = resolve_topics(recording.get("profiles"), profile)
    if not topics:
        raise ValueError(f"recording profile '{profile}' contains no topics")

    output = recording.get("output", {})
    if not isinstance(output, dict):
        raise ValueError("recording.output must be a mapping")
    output_root = expanded_path(
        os.environ.get("DART_BAG_ROOT", output.get("root", "rosbag")), config_path
    )
    output_root.mkdir(parents=True, exist_ok=True)

    lock_file = (output_root / ".recorder.lock").open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError(f"another recorder is already using {output_root}") from error
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()

    storage = recording.get("storage", {})
    if not isinstance(storage, dict):
        raise ValueError("recording.storage must be a mapping")
    storage_id = storage.get("storage_id", "mcap")
    if storage_id not in ("mcap", "sqlite3"):
        raise ValueError("recording.storage.storage_id must be 'mcap' or 'sqlite3'")
    recover_interrupted_sessions(output_root, storage_id)

    session_path = unique_session_path(output_root)
    session_path.mkdir()
    active_marker = session_path / ".recording"
    copy_configuration_snapshot(config_path, recording, session_path / "config")

    bag_path = session_path / "bag"
    command = build_command(recording, topics, bag_path)
    manifest_path = session_path / "manifest.yaml"
    start_time = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    manifest = {
        "session": session_path.name,
        "status": "starting",
        "started_at": start_time,
        "finished_at": None,
        "hostname": socket.gethostname(),
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "git": git_state(config_path),
        "profile": profile,
        "topics": topics,
        "bag_path": "bag",
        "command": command,
        "exit_code": None,
    }
    write_manifest(manifest_path, manifest)
    active_marker.touch()

    child = None
    stop_requested = False
    stop_requested_at = None
    termination_stage = 0

    def request_stop(signum, _frame):
        nonlocal stop_requested, stop_requested_at
        if not stop_requested:
            stop_requested = True
            stop_requested_at = time.monotonic()
            print(f"Stopping rosbag recorder after signal {signum}...", flush=True)
            signal_process_group(child, signal.SIGINT)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    exit_code = 1
    try:
        print(f"Recording profile '{profile}' to {session_path}", flush=True)
        print(f"Recording {len(topics)} topics with rosbag2", flush=True)
        child = subprocess.Popen(command, start_new_session=True)
        if stop_requested:
            signal_process_group(child, signal.SIGINT)
        manifest["status"] = "recording"
        write_manifest(manifest_path, manifest)
        while child.poll() is None:
            if (
                stop_requested
                and stop_requested_at is not None
                and time.monotonic() - stop_requested_at > 15.0
            ):
                if termination_stage == 0:
                    print("Rosbag recorder did not stop in 15 seconds; terminating it", flush=True)
                    signal_process_group(child, signal.SIGTERM)
                    termination_stage = 1
                    stop_requested_at = time.monotonic()
                else:
                    print("Rosbag recorder did not terminate; killing it", flush=True)
                    signal_process_group(child, signal.SIGKILL)
                    stop_requested_at = None
            time.sleep(0.2)
        exit_code = child.returncode
        if stop_requested and exit_code in (0, -signal.SIGINT, -signal.SIGTERM):
            exit_code = 0
    finally:
        manifest["status"] = "complete" if exit_code == 0 else "failed"
        manifest["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        manifest["exit_code"] = exit_code
        write_manifest(manifest_path, manifest)
        active_marker.unlink(missing_ok=True)
        lock_file.close()

    return exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print(f"record_bag: {error}", file=sys.stderr)
        sys.exit(1)
