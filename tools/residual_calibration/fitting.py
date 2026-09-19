"""测量校验、残差拟合与独立验证。角度界面单位为度。"""

import hashlib
import json
from dataclasses import asdict, dataclass

import numpy as np
import pandas as pd

TEXT_COLUMNS = ["sample_id", "session", "split", "notes"]
NUMBER_COLUMNS = [
    "distance_calc_m",
    "distance_ref_m",
    "yaw_calc_deg",
    "yaw_ref_deg",
    "sample_count",
    "distance_std_m",
    "yaw_std_deg",
]
COLUMNS = ["sample_id", "session", "split", *NUMBER_COLUMNS, "notes"]
MODELS = {"identity": "不补偿", "offset": "常量修正", "linear": "线性修正", "quadratic": "二次修正"}
SPLITS = {"train": "拟合", "validation": "验证", "excluded": "排除"}
DIMENSIONS = {
    "distance": ("distance_calc_m", "distance_ref_m", "距离", "m"),
    "yaw": ("yaw_calc_deg", "yaw_ref_deg", "角度", "°"),
}


def frame_from_records(records):
    frame = pd.DataFrame(records)
    for column in COLUMNS:
        if column not in frame:
            frame[column] = (
                "train" if column == "split" else ("" if column in TEXT_COLUMNS else np.nan)
            )
    frame = frame[COLUMNS].copy()
    for column in TEXT_COLUMNS:
        frame[column] = frame[column].fillna("").astype(str)
    for column in NUMBER_COLUMNS:
        try:
            frame[column] = pd.to_numeric(frame[column].replace("", np.nan), errors="raise").astype(
                float
            )
        except (ValueError, TypeError) as error:
            raise ValueError(f"{column} 包含非数字，请修正后导入。") from error
    if np.isinf(frame[NUMBER_COLUMNS].to_numpy()).any():
        raise ValueError("数字不能为无穷大。")
    return frame


def records_from_frame(frame):
    # JSON 不允许 NaN，空单元格使用 null 保存。
    return json.loads(frame.to_json(orient="records", double_precision=15, force_ascii=False))


def validate(frame):
    errors, notices = [], []
    if len(frame) > 10000:
        errors.append("一次最多处理 10000 行。")
    ids = frame["sample_id"].str.strip()
    if (ids == "").any():
        errors.append("每行都需要测量点编号。")
    if ids.duplicated().any():
        errors.append("测量点编号必须唯一；重复采集请使用不同编号。")
    for _, row in frame.iterrows():
        label = row["sample_id"] or "未编号"
        if row["split"] not in SPLITS:
            errors.append(f"{label}：用途必须为 train、validation 或 excluded。")
        for column in NUMBER_COLUMNS:
            value = row[column]
            if pd.notna(value) and not np.isfinite(value):
                errors.append(f"{label}：{column} 不能为无穷大。")
        for column in ("distance_calc_m", "distance_ref_m"):
            if pd.notna(row[column]) and row[column] <= 0:
                errors.append(f"{label}：距离必须大于 0。")
        for column in ("yaw_calc_deg", "yaw_ref_deg"):
            if pd.notna(row[column]) and abs(row[column]) > 180:
                errors.append(f"{label}：角度必须在 [-180, 180] 度内。")
        for column in ("distance_std_m", "yaw_std_deg"):
            if pd.notna(row[column]) and row[column] < 0:
                errors.append(f"{label}：标准差不能为负数。")
        count = row["sample_count"]
        if pd.notna(count) and (count < 1 or not np.isfinite(count) or count % 1):
            errors.append(f"{label}：帧数必须为正整数。")
        for calc, ref, title, _ in DIMENSIONS.values():
            if pd.isna(row[calc]) != pd.isna(row[ref]) and row["split"] != "excluded":
                notices.append(f"{label}：{title}缺少计算值或参考值，此行不参与该项拟合。")
    return list(dict.fromkeys(errors)), notices


def wrap_degrees(value):
    return (np.asarray(value) + 180.0) % 360.0 - 180.0


def fingerprint(records, settings):
    payload = json.dumps(
        {"records": records, "settings": settings},
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode()).hexdigest()


@dataclass
class FitResult:
    dimension: str
    model: str
    coefficients: list[float]  # 完整映射，按常数项到最高次项排列。
    input_min: float
    input_max: float
    train_count: int
    validation_count: int
    warnings: list[str]

    def predict(self, values):
        output = np.polynomial.polynomial.polyval(values, self.coefficients)
        return wrap_degrees(output) if self.dimension == "yaw" else output

    def to_dict(self):
        return asdict(self)


def paired(frame, dimension, split=None):
    calc, ref, _, _ = DIMENSIONS[dimension]
    mask = frame[calc].notna() & frame[ref].notna()
    if split is not None:
        mask &= frame["split"] == split
    return frame.loc[mask].copy()


def fit_dimension(frame, dimension, model):
    errors, _ = validate(frame)
    if errors:
        raise ValueError("\n".join(errors))
    if model not in MODELS or (dimension == "yaw" and model == "quadratic"):
        raise ValueError("不支持的模型。")
    train = paired(frame, dimension, "train")
    calc, ref, _, _ = DIMENSIONS[dimension]
    x, y = train[calc].to_numpy(), train[ref].to_numpy()
    minimum = {"identity": 1, "offset": 2, "linear": 3, "quadratic": 4}[model]
    if len(x) < minimum:
        raise ValueError(f"{MODELS[model]}至少需要 {minimum} 个完整拟合点。")
    # 角度参考值放到计算值附近的分支，避免跨 ±180° 的残差跳变。
    residual = wrap_degrees(y - x) if dimension == "yaw" else y - x
    if dimension == "yaw":
        radians = np.deg2rad(residual)
        sine, cosine = np.mean(np.sin(radians)), np.mean(np.cos(radians))
        if np.hypot(sine, cosine) < 1e-6:
            raise ValueError("角度残差方向不明确，请检查角度正方向和异常记录。")
        center = np.rad2deg(np.arctan2(sine, cosine))
        residual = center + wrap_degrees(residual - center)
    target = x + residual
    if model == "identity":
        coefficients = [0.0, 1.0]
    elif model == "offset":
        coefficients = [float(np.mean(residual)), 1.0]
    else:
        degree = 1 if model == "linear" else 2
        if len(np.unique(x)) < degree + 1 or np.ptp(x) <= 1e-10 * max(1.0, np.max(np.abs(x))):
            raise ValueError("计算值的变化范围不足，无法识别该模型；请增加不同位置的数据。")
        scaled = (x - np.mean(x)) / np.ptp(x)
        if np.linalg.cond(np.polynomial.polynomial.polyvander(scaled, degree)) > 1e8:
            raise ValueError("数据分布过于集中，拟合不可靠。请增加覆盖范围。")
        coefficients = np.polynomial.Polynomial.fit(x, target, degree).convert().coef.tolist()
    if not np.all(np.isfinite(coefficients)):
        raise ValueError("拟合产生非有限系数，请检查数据范围。")
    validation = paired(frame, dimension, "validation")
    warnings = []
    if validation.empty:
        warnings.append("仅拟合效果，尚未独立验证。")
    elif ((validation[calc] < min(x)) | (validation[calc] > max(x))).any():
        warnings.append("部分验证点超出拟合输入范围，报告中将单独标记。")
    if not validation.empty and set(train.session) & set(validation.session):
        warnings.append("拟合与验证含相同批次；请确认并非同一位置相邻帧拆分。")
    if dimension == "distance":
        poly = np.polynomial.Polynomial(coefficients)
        critical = [float(min(x)), float(max(x))]
        for root in poly.deriv().roots():
            if np.isreal(root) and min(x) <= float(np.real(root)) <= max(x):
                critical.append(float(np.real(root)))
        if min(poly(np.asarray(critical))) <= 0:
            raise ValueError("修正距离在拟合范围内出现非正值，请更换模型或检查数据。")
        if min(poly.deriv()(np.array([min(x), max(x)]))) <= 0:
            raise ValueError("距离映射在拟合范围内不单调递增，请更换模型或检查数据。")
    return FitResult(
        dimension,
        model,
        coefficients,
        float(min(x)),
        float(max(x)),
        len(train),
        len(validation),
        warnings,
    )


def error_values(prediction, reference, dimension):
    delta = np.asarray(prediction) - np.asarray(reference)
    return wrap_degrees(delta) if dimension == "yaw" else delta


def metrics(errors):
    values = np.asarray(errors, dtype=float)
    if not len(values):
        return {"count": 0, "bias": None, "mae": None, "rmse": None, "max_abs": None, "std": None}
    return {
        "count": len(values),
        "bias": float(np.mean(values)),
        "mae": float(np.mean(np.abs(values))),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "max_abs": float(np.max(np.abs(values))),
        "std": float(np.std(values)),
    }


def evaluate(frame, result):
    calc, ref, _, _ = DIMENSIONS[result.dimension]
    table = paired(frame, result.dimension)
    table["in_range"] = table[calc].between(result.input_min, result.input_max)
    table["model_prediction"] = result.predict(table[calc].to_numpy())
    table["corrected"] = np.where(table.in_range, table.model_prediction, table[calc])
    table["error_before"] = error_values(table[calc], table[ref], result.dimension)
    table["error_after"] = error_values(table.corrected, table[ref], result.dimension)
    return table


def metric_rows(frame, result):
    table = evaluate(frame, result)
    rows = []
    for split in ("train", "validation"):
        part = table[table.split == split]
        for stage, column in (("before", "error_before"), ("after", "error_after")):
            rows.append({"split": split, "stage": stage, **metrics(part[column])})
    return rows


def comparison(frame, dimension):
    rows = []
    for model in MODELS:
        if dimension == "yaw" and model == "quadratic":
            continue
        try:
            fit = fit_dimension(frame, dimension, model)
            for metric in metric_rows(frame, fit):
                if metric["stage"] == "after":
                    rows.append({"model": model, **metric, "reason": ""})
        except ValueError as error:
            rows.append({"model": model, "split": "train", "reason": str(error)})
    return pd.DataFrame(rows)


def formula(result):
    terms = [
        f"{value:.10g}" if power == 0 else f"({value:.10g})·x" + (f"^{power}" if power > 1 else "")
        for power, value in enumerate(result.coefficients)
    ]
    text = " + ".join(terms)
    return f"y = wrap_deg({text})" if result.dimension == "yaw" else f"y = {text}"
