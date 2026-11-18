"""Execute the notebook; optionally regenerate its report and four figures.

Source ROS first, then run with the analysis virtual environment:
.venv/bin/python analysis/run_target_impact_analysis.py --export-report
"""

import argparse
import os
import sys
from pathlib import Path

import nbformat
from nbclient import NotebookClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-report", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    path = root / "analysis/notebooks/target_impact_analysis.ipynb"
    # The generic python3 kernelspec invokes `python`; use this runner's environment.
    os.environ["PATH"] = str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"]
    os.environ["DART_ANALYSIS_SAVE_FIGURES"] = "1" if args.export_report else "0"
    notebook = nbformat.read(path, as_version=4)

    def progress(cell_index, **_):
        print(f"Executed cell {cell_index + 1}/{len(notebook.cells)}", flush=True)

    client = NotebookClient(
        notebook,
        timeout=300,
        kernel_name="python3",
        resources={"metadata": {"path": str(root)}},
        on_cell_executed=progress,
    )
    client.execute()
    nbformat.write(notebook, path)
    print(f"Updated {path.relative_to(root)}")


if __name__ == "__main__":
    main()
