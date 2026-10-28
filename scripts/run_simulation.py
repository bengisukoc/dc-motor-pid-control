#!/usr/bin/env python3
"""Compare P, PI and PID motor-speed controllers."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib.pyplot as plt
import pandas as pd

from motor_control import PIDGains, response_metrics, simulate


def main() -> None:
    controllers = [
        PIDGains(0.25, 0.0, 0.0, "P"),
        PIDGains(0.22, 1.2, 0.0, "PI"),
        PIDGains(0.28, 1.4, 0.0015, "PID"),
    ]
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    results = {c.name: simulate(c) for c in controllers}
    metrics = pd.DataFrame({name: response_metrics(frame) for name, frame in results.items()}).T
    metrics.index.name = "Controller"
    metrics.to_csv(output / "controller_metrics.csv", float_format="%.4f")

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    colors = {"P": "#6b7280", "PI": "#2563eb", "PID": "#dc2626"}
    for name, frame in results.items():
        axes[0].plot(frame["Time (s)"], frame["Speed (rad/s)"], label=name,
                     color=colors[name], linewidth=1.6)
        axes[1].plot(frame["Time (s)"], frame["Voltage (V)"], label=name,
                     color=colors[name], linewidth=1.3)
        axes[2].plot(frame["Time (s)"], frame["Current (A)"], label=name,
                     color=colors[name], linewidth=1.3)
    axes[0].plot(next(iter(results.values()))["Time (s)"],
                 next(iter(results.values()))["Target speed (rad/s)"], "k--", label="Target")
    for ax in axes:
        ax.axvline(2.50, color="#111827", linestyle=":", alpha=0.7)
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Speed (rad/s)")
    axes[1].set_ylabel("Voltage (V)")
    axes[2].set_ylabel("Current (A)")
    axes[2].set_xlabel("Time (s)")
    axes[0].legend(ncol=4, frameon=False)
    axes[0].set_title("DC motor speed-control comparison", loc="left", weight="bold")
    fig.text(0.72, 0.66, "Load disturbance applied", fontsize=9)
    fig.tight_layout()
    fig.savefig(output / "controller_comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

    best = metrics["Final absolute error (rad/s)"].idxmin()
    summary = [
        "# Controller comparison results", "",
        "The simulation compares P, PI and PID control for a 150 rad/s speed command and a",
        "0.20 N m load disturbance at 2.50 s.", "", "## Metrics", "",
        "```text", metrics.round(4).to_string(na_rep="not reached"), "```", "", "## Main finding", "",
        f"**{best} control produced the smallest final absolute speed error** in this parameter study.",
        "These gains are illustrative; a real design would require system identification, safety margins and hardware testing.",
    ]
    (output / "analysis_summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(metrics.round(4).to_string(na_rep="not reached"))
    print(f"\nResults saved in {output}")


if __name__ == "__main__":
    main()

