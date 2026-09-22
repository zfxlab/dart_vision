"""项目持久化与可追溯的模型导出。"""

import io
import json
import math
import os
import tempfile
import uuid
from datetime import UTC, datetime
from pathlib import Path

import pandas as pd
import yaml

from fitting import (
    DIMENSIONS,
    MODELS,
    evaluate,
    fingerprint,
    frame_from_records,
    metric_rows,
    records_from_frame,
)

SCHEMA_VERSION = 1


def timestamp():
    return datetime.now(UTC).isoformat(timespec="seconds")


def data_directory():
    return Path(os.environ.get("DART_CALIBRATION_DATA_DIR", Path(__file__).parent / ".local_data"))


def new_project(name="未命名测量项目"):
    return {
        "schema_version": SCHEMA_VERSION,
        "project_id": uuid.uuid4().hex,
        "name": name,
        "reference_frame": "launcher_frame",
        "created_at": timestamp(),
        "updated_at": timestamp(),
        "measurements": [],
        "settings": {"distance": "linear", "yaw": "offset"},
    }


def project_bytes(project):
    return json.dumps(project, indent=2, ensure_ascii=False, allow_nan=False).encode("utf-8")


def load_project(data):
    try:
        project = json.loads(data)
        if not isinstance(project, dict) or project.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("项目文件版本不支持。")
        if project.get("reference_frame") != "launcher_frame":
            raise ValueError("本工具只接受 launcher_frame 测量项目。")
        records = project["measurements"]
        if not isinstance(records, list) or len(records) > 10000:
            raise ValueError("项目数据必须为表格，且不超过 10000 行。")
        if any(not isinstance(record, dict) for record in records):
            raise ValueError("项目测量记录格式错误。")
        settings = project["settings"]
        if not isinstance(settings, dict):
            raise ValueError("项目模型设置无效。")
        for dimension in DIMENSIONS:
            model = settings.get(dimension)
            if model not in MODELS or (dimension == "yaw" and model == "quadratic"):
                raise ValueError("项目模型设置无效。")
        name = project.get("name", "未命名测量项目")
        if not isinstance(name, str):
            raise ValueError("项目名称必须为文本。")
        clean = new_project(name)
        clean["created_at"] = str(project.get("created_at", clean["created_at"]))
        clean["settings"] = {key: settings[key] for key in DIMENSIONS}
        clean["measurements"] = records_from_frame(frame_from_records(records))
        # 打开外部项目创建新的本地副本，避免覆盖同名草稿。
        return clean
    except (KeyError, TypeError, json.JSONDecodeError, UnicodeDecodeError) as error:
        raise ValueError("项目文件无效或不完整。") from error


def read_csv(data):
    try:
        frame = pd.read_csv(
            io.BytesIO(data), encoding="utf-8-sig", dtype=str, keep_default_na=False
        )
    except (ValueError, UnicodeDecodeError, pd.errors.ParserError) as error:
        raise ValueError("无法读取 CSV，请使用 UTF-8 编码和英文列名。") from error
    if "sample_id" not in frame:
        raise ValueError("CSV 缺少 sample_id 列，可先下载示例模板。")
    if not any(calc in frame and ref in frame for calc, ref, _, _ in DIMENSIONS.values()):
        raise ValueError("CSV 至少需要一组计算值和参考值列。")
    if len(frame) > 10000:
        raise ValueError("一次最多导入 10000 行。")
    return frame_from_records(frame.to_dict("records"))


def csv_bytes(frame):
    return frame.to_csv(index=False).encode("utf-8-sig")


def save_draft(project, directory=None):
    directory = Path(directory or data_directory())
    directory.mkdir(parents=True, exist_ok=True)
    identifier = uuid.UUID(project["project_id"]).hex
    payload = project_bytes(project)
    # 原子替换，避免进程中断留下半个 JSON 文件。
    with tempfile.NamedTemporaryFile(dir=directory, suffix=".tmp", delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(payload)
    try:
        temporary.replace(directory / f"{identifier}.json")
    finally:
        temporary.unlink(missing_ok=True)


def list_drafts(directory=None):
    directory = Path(directory or data_directory())
    drafts = []
    for path in directory.glob("*.json"):
        try:
            project = load_project(path.read_bytes())
            drafts.append((path, project["name"]))
        except (OSError, ValueError):
            continue
    return sorted(drafts, key=lambda item: item[0].stat().st_mtime, reverse=True)


def compensation_document(project, frame, results, fitted_fingerprint):
    current = fingerprint(records_from_frame(frame), project["settings"])
    if current != fitted_fingerprint:
        raise ValueError("数据或模型已变化，请重新拟合后导出。")
    if not results:
        raise ValueError("没有可导出的模型。")
    models = {}
    for dimension, result in results.items():
        scale = math.pi / 180 if dimension == "yaw" else 1.0
        coefficients = [
            float(value * scale ** (1 - power)) for power, value in enumerate(result.coefficients)
        ]
        models[dimension] = {
            "enabled": False,
            "model": result.model,
            "mapping": "corrected = sum(coefficients[k] * raw**k)",
            "coefficient_order": "ascending",
            "coefficients": coefficients,
            "input_unit": "rad" if dimension == "yaw" else "m",
            "output_unit": "rad" if dimension == "yaw" else "m",
            "wrap_output_to_pi": dimension == "yaw",
            "input_range": [result.input_min * scale, result.input_max * scale],
            "outside_range_policy": "use_raw",
            "train_count": result.train_count,
            "validation_count": result.validation_count,
            "validation_status": "holdout_evaluated"
            if result.validation_count
            else "not_validated",
            "metrics_unit": DIMENSIONS[dimension][3],
            "metrics": metric_rows(frame, result),
            "warnings": result.warnings,
        }
    return {
        "schema_version": 1,
        "kind": "dart_residual_compensation",
        "project": project["name"],
        "created_at": timestamp(),
        "reference_frame": "launcher_frame",
        "source_fingerprint": current,
        "angle_convention": "right_positive_before_dart_offset",
        "application_stage": "after_tf_before_temporal_filter_and_dart_offset",
        "models": models,
    }


def yaml_bytes(document):
    return yaml.safe_dump(document, allow_unicode=True, sort_keys=False).encode("utf-8")


def corrected_frame(frame, results):
    output = frame.copy()
    for dimension, result in results.items():
        evaluated = evaluate(frame, result)
        for column in ("corrected", "error_before", "error_after", "in_range"):
            output[f"{dimension}_{column}"] = evaluated[column]
    return output
