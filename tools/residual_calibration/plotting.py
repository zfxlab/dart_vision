"""交互图表、独立 HTML 报告和静态图片。"""

import html
import io
import json
import zipfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go

from fitting import DIMENSIONS, SPLITS, evaluate, formula, metric_rows
from storage import corrected_frame, csv_bytes, project_bytes, yaml_bytes

COLORS = {"train": "#2563eb", "validation": "#059669", "excluded": "#94a3b8"}
SYMBOLS = {"train": "circle", "validation": "diamond", "excluded": "x"}


def figures(frame, result):
    table = evaluate(frame, result)
    calc, ref, title, unit = DIMENSIONS[result.dimension]
    plots = {
        "agreement": go.Figure(),
        "residual": go.Figure(),
        "mapping": go.Figure(),
        "conditions": go.Figure(),
    }
    cross = "yaw_calc_deg" if result.dimension == "distance" else "distance_calc_m"
    cross_label = "计算角度 / °" if result.dimension == "distance" else "计算距离 / m"
    for split, label in SPLITS.items():
        part = table[table.split == split]
        if part.empty:
            continue
        custom = part[["sample_id", "session", "split", "notes", "in_range"]].to_numpy()
        common = {
            "mode": "markers",
            "customdata": custom,
            "hovertemplate": "编号 %{customdata[0]}<br>批次 %{customdata[1]}<br>x=%{x:.5f}<br>y=%{y:.5f}<br>拟合范围内 %{customdata[4]}<extra>%{fullData.name}</extra>",
        }
        marker = {"color": COLORS[split], "symbol": SYMBOLS[split], "size": 9}
        for stage, column, error, symbol in (
            ("原始", calc, "error_before", "circle-open"),
            ("修正", "corrected", "error_after", SYMBOLS[split]),
        ):
            plots["agreement"].add_trace(
                go.Scatter(
                    x=part[ref],
                    y=part[column],
                    name=f"{label} · {stage}",
                    marker={**marker, "symbol": symbol},
                    **common,
                )
            )
            plots["residual"].add_trace(
                go.Scatter(
                    x=part[calc],
                    y=part[error],
                    name=f"{label} · {stage}",
                    marker={**marker, "symbol": symbol},
                    **common,
                )
            )
        plots["mapping"].add_trace(
            go.Scatter(x=part[calc], y=part[ref], name=label, marker=marker, **common)
        )
        present = part[cross].notna()
        if present.any():
            plots["conditions"].add_trace(
                go.Scatter(
                    x=part.loc[present, cross],
                    y=part.loc[present, "error_after"],
                    name=label,
                    marker=marker,
                    **{**common, "customdata": custom[present.to_numpy()]},
                )
            )
    if not table.empty:
        lo = min(table[ref].min(), table[calc].min(), table.corrected.min())
        hi = max(table[ref].max(), table[calc].max(), table.corrected.max())
        plots["agreement"].add_trace(
            go.Scatter(
                x=[lo, hi],
                y=[lo, hi],
                mode="lines",
                name="理想 y=x",
                line={"color": "#64748b", "dash": "dash"},
            )
        )
    grid = np.linspace(result.input_min, result.input_max, 200)
    plots["mapping"].add_trace(
        go.Scatter(
            x=grid,
            y=result.predict(grid),
            mode="lines",
            name="选定模型（拟合范围）",
            line={"color": "#f97316", "width": 3},
        )
    )
    plots["mapping"].add_trace(
        go.Scatter(
            x=grid, y=grid, mode="lines", name="不补偿", line={"color": "#64748b", "dash": "dash"}
        )
    )
    plots["residual"].add_hline(y=0, line_dash="dash", line_color="#64748b")
    plots["conditions"].add_hline(y=0, line_dash="dash", line_color="#64748b")
    labels = {
        "agreement": (f"{title}：计算值与参考值", f"参考值 / {unit}", f"原始或修正值 / {unit}"),
        "residual": ("补偿前后残差", f"计算值 / {unit}", f"计算 − 参考 / {unit}"),
        "mapping": ("完整映射与有效采样范围", f"计算值 / {unit}", f"修正值 / {unit}"),
        "conditions": ("修正残差与另一测量量的关系", cross_label, f"修正 − 参考 / {unit}"),
    }
    for key, figure in plots.items():
        heading, xlabel, ylabel = labels[key]
        figure.update_layout(
            title=heading,
            xaxis_title=xlabel,
            yaxis_title=ylabel,
            template="plotly_white",
            height=400,
            margin={"l": 20, "r": 20, "t": 50, "b": 30},
            legend={"orientation": "h", "y": -0.2},
            clickmode="event+select",
        )
    return plots


def metric_table(frame, result):
    rows = pd.DataFrame(metric_rows(frame, result))
    rows["split"] = rows.split.map(SPLITS)
    rows["stage"] = rows.stage.map({"before": "补偿前", "after": "补偿后"})
    return rows.rename(
        columns={
            "split": "用途",
            "stage": "阶段",
            "count": "点数",
            "bias": "平均误差",
            "mae": "MAE",
            "rmse": "RMSE",
            "max_abs": "最大绝对误差",
            "std": "误差标准差",
        }
    )


def report_html(project, frame, results, document):
    pieces = [
        "<!doctype html><html lang='zh-CN'><meta charset='utf-8'><title>残差标定报告</title>",
        "<style>body{font:15px sans-serif;max-width:1200px;margin:40px auto;padding:0 24px;color:#1e293b}table{border-collapse:collapse}td,th{border:1px solid #cbd5e1;padding:8px}pre{white-space:pre-wrap}h2{margin-top:40px}</style>",
        f"<h1>{html.escape(project['name'])}</h1><p>坐标系 launcher_frame；角度向右为正，未加入 dart_offset_rad。</p>",
        "<p>本报告为离线测量拟合。验证结果仅覆盖记录条件，采样范围不等于精度保证。范围外记录保留原始值并计入验证指标。</p>",
    ]
    include_js = True
    for dimension, result in results.items():
        pieces.append(
            f"<h2>{DIMENSIONS[dimension][2]}补偿</h2><pre>{html.escape(formula(result))}</pre>"
        )
        pieces.extend(f"<p>{html.escape(warning)}</p>" for warning in result.warnings)
        pieces.append(
            metric_table(frame, result).to_html(index=False, float_format=lambda x: f"{x:.6g}")
        )
        for figure in figures(frame, result).values():
            pieces.append(figure.to_html(full_html=False, include_plotlyjs=include_js))
            include_js = False
    pieces.append("<h2>原始记录（含排除项）</h2>" + frame.to_html(index=False, escape=True))
    pieces.append(
        "<h2>导出模型元数据</h2><pre>"
        + html.escape(json.dumps(document, ensure_ascii=False, indent=2))
        + "</pre></html>"
    )
    return "\n".join(pieces).encode("utf-8")


def static_png(frame, result):
    """图片使用英文轴标签，避免部署机器缺少中文字体。"""
    table = evaluate(frame, result)
    calc, ref, _, unit = DIMENSIONS[result.dimension]
    unit = "deg" if unit == "°" else unit
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for split in SPLITS:
        part = table[table.split == split]
        if part.empty:
            continue
        color = COLORS[split]
        axes[0, 0].scatter(
            part[ref], part[calc], facecolors="none", edgecolors=color, label=f"{split} raw"
        )
        axes[0, 0].scatter(
            part[ref], part.corrected, color=color, marker="x", label=f"{split} corrected"
        )
        axes[0, 1].scatter(
            part[calc], part.error_before, facecolors="none", edgecolors=color, label=f"{split} raw"
        )
        axes[0, 1].scatter(
            part[calc], part.error_after, color=color, marker="x", label=f"{split} corrected"
        )
        axes[1, 0].scatter(part[calc], part[ref], color=color, label=split)
        cross = "yaw_calc_deg" if result.dimension == "distance" else "distance_calc_m"
        axes[1, 1].scatter(part[cross], part.error_after, color=color, label=split)
    grid = np.linspace(result.input_min, result.input_max, 200)
    axes[0, 0].plot(grid, grid, "--", color="gray")
    axes[0, 1].axhline(0, linestyle="--", color="gray")
    axes[1, 0].plot(grid, result.predict(grid), color="#f97316", label="fit (training range)")
    axes[1, 1].axhline(0, linestyle="--", color="gray")
    labels = [
        ("Reference agreement", f"Reference ({unit})", f"Raw / corrected ({unit})"),
        ("Residuals", f"Raw ({unit})", f"Estimate - reference ({unit})"),
        ("Correction mapping", f"Raw ({unit})", f"Corrected ({unit})"),
        (
            "Cross-variable residual",
            "Raw yaw (deg)" if result.dimension == "distance" else "Raw distance (m)",
            f"Residual ({unit})",
        ),
    ]
    for axis, (title, xlabel, ylabel) in zip(axes.flat, labels, strict=True):
        axis.set(title=title, xlabel=xlabel, ylabel=ylabel)
        axis.grid(alpha=0.2)
        axis.legend(fontsize=7)
    stream = io.BytesIO()
    figure.savefig(stream, format="png", dpi=160)
    plt.close(figure)
    return stream.getvalue()


def export_bundle(project, frame, results, document):
    stream = io.BytesIO()
    with zipfile.ZipFile(stream, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("project.json", project_bytes(project))
        archive.writestr("measurements.csv", csv_bytes(frame))
        archive.writestr("corrected.csv", csv_bytes(corrected_frame(frame, results)))
        archive.writestr("compensation.yaml", yaml_bytes(document))
        archive.writestr("report.html", report_html(project, frame, results, document))
        for dimension, result in results.items():
            archive.writestr(f"plots/{dimension}.png", static_png(frame, result))
        archive.writestr(
            "README.txt",
            "compensation.yaml 是离线模型，不会自动写入 ROS 配置。\n完整映射使用升幂系数。角度模型输入输出为弧度。\n当前 dart_aiming 尚未实现此模型格式的加载，需要接入后显式启用。\n所有模型默认 enabled=false；范围外策略为使用原始值。\n",
        )
    return stream.getvalue()
