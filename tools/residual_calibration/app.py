"""本地测量记录与残差拟合界面。"""

import sys
from pathlib import Path

# 支持从任意目录通过 streamlit run app.py 启动。
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import streamlit as st
from tools.residual_calibration.fitting import (
    COLUMNS,
    DIMENSIONS,
    MODELS,
    SPLITS,
    comparison,
    fingerprint,
    fit_dimension,
    formula,
    frame_from_records,
    records_from_frame,
    validate,
)
from tools.residual_calibration.plotting import export_bundle, figures, metric_table
from tools.residual_calibration.storage import (
    compensation_document,
    csv_bytes,
    list_drafts,
    load_project,
    new_project,
    project_bytes,
    read_csv,
    save_draft,
    timestamp,
    yaml_bytes,
)

EXAMPLE = Path(__file__).parent / "examples" / "measurements.csv"


def open_in_session(project):
    st.session_state.project = project
    st.session_state.editor_base = frame_from_records(project["measurements"])
    st.session_state.editor_generation = st.session_state.get("editor_generation", 0) + 1
    st.session_state.project_name = project["name"]
    for dimension in DIMENSIONS:
        st.session_state[f"model_{dimension}"] = project["settings"][dimension]
    st.session_state.fit_results = {}
    st.session_state.fit_signature = None
    st.session_state.saved_signature = None
    st.session_state.exports = None
    st.session_state.pop("selected_sample", None)
    st.session_state.selection_seen = {}


def replace_table(frame):
    st.session_state.project["measurements"] = records_from_frame(frame)
    st.session_state.editor_base = frame.copy()
    st.session_state.editor_generation += 1
    st.rerun()


def main():
    st.set_page_config(page_title="绿灯残差标定", page_icon="📐", layout="wide")
    st.markdown(
        """<style>
    .block-container {padding-top:2rem;max-width:1600px}
    [data-testid="stMetric"] {background:#f1f5f9;padding:12px;border-radius:8px}
    </style>""",
        unsafe_allow_html=True,
    )
    if "project" not in st.session_state:
        open_in_session(new_project())
    state = st.session_state
    st.title("绿灯残差标定")
    st.caption("记录测量 → 比较模型 → 独立验证 → 导出补偿结果")

    with st.sidebar:
        st.header("测量项目")
        if st.button("新建项目", width="stretch"):
            open_in_session(new_project())
            st.rerun()
        with st.expander("打开项目 / 导入数据"):
            upload = st.file_uploader("项目 JSON 或测量 CSV", type=["json", "csv"])
            if st.button("打开上传文件", disabled=upload is None):
                try:
                    if upload.name.lower().endswith(".json"):
                        project = load_project(upload.getvalue())
                    else:
                        project = new_project(Path(upload.name).stem)
                        project["measurements"] = records_from_frame(read_csv(upload.getvalue()))
                    open_in_session(project)
                    st.rerun()
                except ValueError as error:
                    st.error(str(error))
            drafts = list_drafts()
            if drafts:
                draft_names = {str(path): f"{name} · {path.stem[:6]}" for path, name in drafts}
                choice = st.selectbox(
                    "本地草稿", list(draft_names), format_func=draft_names.get, key="draft_path"
                )
                if st.button("恢复所选草稿"):
                    try:
                        open_in_session(load_project(Path(choice).read_bytes()))
                        st.rerun()
                    except (ValueError, OSError) as error:
                        st.error(str(error))
            if st.button("载入演示数据"):
                project = new_project("演示项目 · 合成数据")
                project["measurements"] = records_from_frame(read_csv(EXAMPLE.read_bytes()))
                open_in_session(project)
                st.rerun()
            st.download_button(
                "下载 CSV 模板与示例", EXAMPLE.read_bytes(), "measurements.csv", "text/csv"
            )
        st.text_input("项目名称", key="project_name")
        st.divider()
        st.subheader("拟合模型")
        st.selectbox("距离模型", list(MODELS), format_func=MODELS.get, key="model_distance")
        st.selectbox(
            "角度模型",
            [key for key in MODELS if key != "quadratic"],
            format_func=MODELS.get,
            key="model_yaw",
        )
        st.caption("使用完整映射：修正值 = f(计算值)。各测量点等权拟合。")

    project = state.project
    project["name"] = state.project_name.strip() or "未命名测量项目"
    project["settings"] = {dimension: state[f"model_{dimension}"] for dimension in DIMENSIONS}
    st.subheader("1 · 测量记录")
    st.caption(
        "每行记录一个位置的一次采集。可在表格底部增加行，距离或角度允许单独填写；排除项仍保留在项目与报告中。"
    )
    columns = {
        "sample_id": st.column_config.TextColumn("编号", required=True),
        "session": st.column_config.TextColumn("批次"),
        "split": st.column_config.SelectboxColumn(
            "用途",
            options=list(SPLITS),
            required=True,
            default="train",
            help="train=拟合，validation=验证，excluded=排除",
        ),
        "distance_calc_m": st.column_config.NumberColumn("计算距离 / m", format="%.5f"),
        "distance_ref_m": st.column_config.NumberColumn("实测距离 / m", format="%.5f"),
        "yaw_calc_deg": st.column_config.NumberColumn("计算角度 / °", format="%.5f"),
        "yaw_ref_deg": st.column_config.NumberColumn("实测角度 / °", format="%.5f"),
        "notes": st.column_config.TextColumn("备注 / 排除原因", width="large"),
    }
    edited = st.data_editor(
        state.editor_base,
        key=f"measurements_{state.editor_generation}",
        num_rows="dynamic",
        column_config=columns,
        column_order=COLUMNS,
        hide_index=True,
        width="stretch",
        height=310,
    )
    frame = frame_from_records(edited.to_dict("records"))
    records = records_from_frame(frame)
    project["measurements"] = records
    signature = fingerprint(records, project["settings"])
    save_signature = (signature, project["name"])
    has_content = not frame.empty or project["name"] != "未命名测量项目"
    if state.saved_signature != save_signature and has_content:
        project["updated_at"] = timestamp()
        try:
            save_draft(project)
            state.saved_signature = save_signature
        except OSError as error:
            st.error(f"草稿保存失败：{error}。可用下方按钮下载项目。")
    save, csv, status = st.columns([1, 1, 3])
    save.download_button(
        "保存项目 JSON",
        project_bytes(project),
        "calibration_project.json",
        "application/json",
        width="stretch",
    )
    csv.download_button(
        "导出测量 CSV", csv_bytes(frame), "measurements.csv", "text/csv", width="stretch"
    )
    status.caption(
        f"草稿已保存 · {project['updated_at']}"
        if state.saved_signature == save_signature
        else ("有未保存修改" if has_content else "尚无测量记录")
    )
    errors, notices = validate(frame)
    for error in errors:
        st.error(error)
    if notices:
        with st.expander(f"{len(notices)} 条不完整记录提示"):
            for notice in notices:
                st.write(notice)

    st.subheader("2 · 拟合与验证")
    if st.button("执行拟合", type="primary", disabled=bool(errors) or frame.empty):
        results, failures = {}, []
        for dimension, (_, _, title, _) in DIMENSIONS.items():
            try:
                results[dimension] = fit_dimension(frame, dimension, project["settings"][dimension])
            except ValueError as error:
                failures.append(f"{title}：{error}")
        state.fit_results = results
        state.fit_signature = signature
        state.fit_failures = failures
        state.exports = None
    stale = bool(state.fit_results) and state.fit_signature != signature
    if stale:
        st.warning("数据或模型设置已修改，旧拟合结果已失效。请重新拟合；导出和试算已暂停。")
        state.exports = None
    if not state.fit_results or stale:
        for failure in state.get("fit_failures", []) if state.fit_signature == signature else []:
            st.warning(failure)
        st.caption("导入记录或载入演示数据后执行拟合。拟合和验证数据请按位置或采集批次划分。")
        return
    for failure in state.get("fit_failures", []):
        st.warning(failure)
    results = state.fit_results
    dimension = st.radio(
        "查看结果", list(results), format_func=lambda key: DIMENSIONS[key][2], horizontal=True
    )
    result = results[dimension]
    st.code(formula(result), language=None)
    st.caption(
        f"输入范围：[{result.input_min:.6g}, {result.input_max:.6g}] {DIMENSIONS[dimension][3]} · 系数为完整映射，不能再加一次原始值。"
    )
    for warning in result.warnings:
        st.warning(warning)
    metrics = metric_table(frame, result)
    st.dataframe(metrics, hide_index=True, width="stretch")
    st.caption(
        "验证指标不参与模型拟合。范围外记录按导出策略保留原始值，计入验证指标；不会扩大模型有效范围。"
    )
    with st.expander("比较所有候选模型"):
        compared = comparison(frame, dimension)
        compared["model"] = compared.model.map(MODELS)
        compared["split"] = compared.split.map(SPLITS)
        st.dataframe(compared, hide_index=True, width="stretch")
        st.caption("没有自动选择最优模型。优先参考独立验证误差，而不是只比较拟合误差。")

    plots = figures(frame, result)
    panels = st.columns(2)
    for index, (key, figure) in enumerate(plots.items()):
        with panels[index % 2]:
            chart_key = f"plot_{dimension}_{key}_{signature}"
            event = st.plotly_chart(
                figure,
                key=chart_key,
                on_select="rerun",
                selection_mode="points",
                width="stretch",
                config={
                    "displaylogo": False,
                    "toImageButtonOptions": {"format": "png", "filename": f"{dimension}_{key}"},
                },
            )
            points = event.selection.points
            if points:
                point = points[-1]
                custom = point.get("customdata")
                token = (chart_key, str(custom), point.get("point_index"))
                if custom and token != state.selection_seen.get(chart_key):
                    state.selected_sample = custom[0]
                    state.selection_seen[chart_key] = token
    st.caption("点击散点可查看对应记录；曲线不是测量点。图表右上角支持保存当前图像。")
    options = frame.sample_id.tolist()
    if state.get("selected_sample") not in options:
        state.selected_sample = options[0]
    selected = st.selectbox("选中的测量记录", options, key="selected_sample")
    selected_rows = frame[frame.sample_id == selected]
    st.dataframe(selected_rows, hide_index=True, width="stretch")
    with st.form("change_usage"):
        usage = st.selectbox("将选中点设为", list(SPLITS), format_func=SPLITS.get)
        reason = st.text_input("备注（排除时必填）")
        if st.form_submit_button("更新用途并重新检查"):
            if usage == "excluded" and not reason.strip():
                st.error("请填写排除原因，原始记录会保留。")
            else:
                mask = frame.sample_id == selected
                frame.loc[mask, "split"] = usage
                if reason.strip():
                    frame.loc[mask, "notes"] = frame.loc[mask, "notes"].apply(
                        lambda old: f"{old} | {reason.strip()}".strip(" |")
                    )
                replace_table(frame)

    st.subheader("3 · 单点试算")
    inputs = st.columns(2)
    for index, (name, fit) in enumerate(results.items()):
        with inputs[index]:
            _, _, title, unit = DIMENSIONS[name]
            raw = st.number_input(
                f"输入计算{title} / {unit}",
                value=float((fit.input_min + fit.input_max) / 2),
                key=f"probe_{name}",
                format="%.6f",
            )
            inside = fit.input_min <= raw <= fit.input_max
            if inside:
                corrected = float(fit.predict(raw))
                delta = corrected - raw
                if name == "yaw":
                    delta = (delta + 180) % 360 - 180
                st.metric(
                    f"修正{title} / {unit}",
                    f"{corrected:.6f}",
                    f"补偿 {delta:+.6f}",
                    delta_color="off",
                )
                st.caption("处于拟合输入范围内；精度仍以独立验证为准。")
            else:
                st.warning(f"超出拟合输入范围，按导出策略保留原始值 {raw:.6f} {unit}。")

    st.subheader("4 · 导出结果")
    st.caption(
        "导出包含原始记录、修正表、PNG 图片和可离线打开的 HTML 报告。YAML 是独立模型文件，默认不启用；当前 dart_aiming 尚未接入该格式。"
    )
    export_key = (signature, project["name"])
    if st.button("生成导出文件"):
        with st.spinner("正在生成报告与图片…"):
            try:
                document = compensation_document(project, frame, results, state.fit_signature)
                state.exports = {
                    "key": export_key,
                    "yaml": yaml_bytes(document),
                    "zip": export_bundle(project, frame, results, document),
                }
            except (ValueError, OSError) as error:
                st.error(str(error))
    exported = state.exports
    if exported and exported["key"] == export_key:
        left, right = st.columns(2)
        left.download_button(
            "下载补偿 YAML",
            exported["yaml"],
            "compensation.yaml",
            "application/yaml",
            width="stretch",
        )
        right.download_button(
            "下载完整结果 ZIP",
            exported["zip"],
            "calibration_result.zip",
            "application/zip",
            width="stretch",
        )


if __name__ == "__main__":
    main()
