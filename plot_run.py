#!/usr/bin/env python3
"""Plot the trajectories of a single planner run.

Same figure style as plot_effect_nondimensionalization.py, but for one run
instead of a with/without-nondimensionalization comparison. The planner invokes
this after each run; it can also be called by hand:

    python3 plot_run.py build/trajectories_intersection_posit16es2_withQuanti.csv
    python3 plot_run.py <csv> --out media/foo.png --title "posit<16,2>"
"""
from pathlib import Path
from typing import Optional
import argparse
import re
import sys

import matplotlib as mpl

mpl.use("Agg")  # no display needed

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

# === IEEE-like font family and sizes (10pt doc) ===
S = {
    "normalsize": 10,
    "small": 9,
    "footnotesize": 8,
    "large": 12,
}
mpl.rcParams.update({
    "text.usetex": False,
    "mathtext.fontset": "stix",
    "axes.titlesize": S["large"],
    "axes.labelsize": S["small"],
    "xtick.labelsize": S["footnotesize"],
    "ytick.labelsize": S["footnotesize"],
    "legend.fontsize": S["footnotesize"],
    "pdf.fonttype": 42, "ps.fonttype": 42,
})

_FONT_CANDIDATES = [
    "Times New Roman",
    "Times",
    "Nimbus Roman",
    "DejaVu Serif",
    "STIXGeneral",
]


def _select_font_property():
    for family in _FONT_CANDIDATES:
        prop = font_manager.FontProperties(family=family)
        try:
            font_manager.findfont(prop, fallback_to_default=False)
        except ValueError:
            continue
        return prop
    return font_manager.FontProperties(family="serif")


_BASE_FONT = _select_font_property()


def font_prop(size_key: str) -> font_manager.FontProperties:
    prop = _BASE_FONT.copy()
    prop.set_size(S[size_key])
    return prop


def plot_trajectories(df: pd.DataFrame, ax) -> None:
    """Draw every vehicle's path plus a box at its starting pose."""
    for vid in df["vehicle_id"].unique():
        vehicle_data = df[df["vehicle_id"] == vid]

        x_vals = vehicle_data["x"].to_numpy()
        y_vals = vehicle_data["y"].to_numpy()
        psi_vals = vehicle_data["psi"].to_numpy()

        ax.plot(x_vals, y_vals, marker="o", markersize=3, label=f"Vehicle {vid}")

        if len(x_vals) > 0:
            x_start, y_start, psi_start = x_vals[0], y_vals[0], psi_vals[0]

            vehicle_length = 4.5  # meters
            vehicle_width = 2.0   # meters

            x_corner = (x_start - (vehicle_length / 2) * np.cos(psi_start)
                        + (vehicle_width / 2) * np.sin(psi_start))
            y_corner = (y_start - (vehicle_length / 2) * np.sin(psi_start)
                        - (vehicle_width / 2) * np.cos(psi_start))

            ax.add_patch(Rectangle(
                (x_corner, y_corner), vehicle_length, vehicle_width,
                angle=np.degrees(psi_start),
                edgecolor="red", facecolor="none", lw=1.5,
            ))


R_SAFE = 2.5        # metres; matches Parameters::r_safe (the detection threshold)
R_MARKER = 1.5      # metres; radius actually drawn, kept small so markers stay readable


def fmt_eps(text: str) -> str:
    """0.031 -> 3.1e-2, 0.00033 -> 3.3e-4."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return text
    if value == 0.0:
        return "0"
    mantissa, exponent = f"{value:.6e}".split("e")
    mantissa = mantissa.rstrip("0").rstrip(".")
    return f"{mantissa}e{int(exponent)}"


def final_cost(df: pd.DataFrame) -> float:
    """Accumulated planning cost: each vehicle's last l, averaged over vehicles."""
    work = df.dropna(subset=["time"])
    return float(work.groupby("vehicle_id")["l"].last().mean())


def mark_collisions(df: pd.DataFrame, ax) -> int:
    """Circle every point where two vehicles are closer than R_SAFE.

    Mirrors the planner's collision constraint, which is violated when the
    squared separation drops below r_safe^2 at the same time step.
    """
    work = df.dropna(subset=["time"])
    collisions = []

    for _, snapshot in work.groupby("time"):
        rows = snapshot[["vehicle_id", "x", "y"]].to_numpy()
        for a in range(len(rows)):
            for b in range(a + 1, len(rows)):
                dx = rows[a][1] - rows[b][1]
                dy = rows[a][2] - rows[b][2]
                if np.hypot(dx, dy) < R_SAFE:
                    collisions.append(((rows[a][1] + rows[b][1]) / 2.0,
                                       (rows[a][2] + rows[b][2]) / 2.0))

    for cx, cy in collisions:
        ax.add_patch(plt.Circle(
            (cx, cy), R_MARKER,
            edgecolor="red", facecolor="none", lw=1.6, zorder=5,
        ))

    return len(collisions)


def style_axes(ax, subplot_title: Optional[str] = None,
               info_lines: Optional[list] = None,
               has_collisions: bool = False) -> None:
    axis_font = font_prop("large")
    title_font = font_prop("large")
    tick_font = font_prop("normalsize")

    ax.set_xlabel("X Position [m]", fontproperties=axis_font)
    ax.set_ylabel("Y Position [m]", fontproperties=axis_font)
    if subplot_title:
        ax.set_title(subplot_title, fontproperties=title_font)

    handles, labels = ax.get_legend_handles_labels()

    if has_collisions:
        # A Circle patch would render as a rectangle in the legend, so use a
        # ring-marker proxy instead.
        handles.append(Line2D([], [], linestyle="none", marker="o",
                              markersize=9, markerfacecolor="none",
                              markeredgecolor="red", markeredgewidth=1.6))
        labels.append("Collision")

    # Run parameters share the legend, on invisible handles so only text shows.
    for line in (info_lines or []):
        handles.append(Line2D([], [], linestyle="none", marker="none"))
        labels.append(line)

    # Legend sits outside the axes so it never covers a trajectory.
    legend = ax.legend(handles, labels, loc="upper left",
                       bbox_to_anchor=(1.02, 1.0),
                       borderaxespad=0.0, frameon=True)
    for text in legend.get_texts():
        text.set_fontproperties(tick_font)

    for tick in ax.get_xticklabels():
        tick.set_fontproperties(tick_font)
    for tick in ax.get_yticklabels():
        tick.set_fontproperties(tick_font)


def derive_title(csv_path: Path) -> str:
    """trajectories_intersection_posit16es3_withQuanti -> Intersection posit16 es3 with Quanti"""
    parts = csv_path.stem.replace("trajectories_", "").split("_")
    words = [parts[0].capitalize()] if parts else ["Run"]

    for part in parts[1:]:
        if part == "withQuanti":
            words.append("with Quanti")
        elif part == "withoutQuanti":
            words.append("without Quanti")
        else:
            # posit16es3 -> "posit16 es3"
            words.append(re.sub(r"(posit\d+)(es\d+)", r"\1 \2", part))

    return " ".join(words)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="trajectories_*.csv from a planner run")
    ap.add_argument("--out", type=Path, default=None,
                    help="output PNG (default: <repo>/media/<csv stem>.png)")
    ap.add_argument("--title", default=None, help="figure title")
    ap.add_argument("--eps", default=None,
                    help="finite-difference step used by the run")
    ap.add_argument("--es", default=None,
                    help="posit exponent size (omit for IEEE runs)")
    ap.add_argument("--dpi", type=int, default=200)
    args = ap.parse_args()

    if not args.csv.exists():
        print(f"plot_run: trajectories file not found: {args.csv}", file=sys.stderr)
        return 1

    df = pd.read_csv(args.csv)
    if df.empty:
        print(f"plot_run: {args.csv} has no rows", file=sys.stderr)
        return 1

    # Default output: <repo>/media/<stem>.png
    out = args.out
    if out is None:
        repo_root = Path(__file__).resolve().parent
        out = repo_root / "media" / f"{args.csv.stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 6))

    plot_trajectories(df, ax)
    n_collisions = mark_collisions(df, ax)

    title = args.title or derive_title(args.csv)

    info_lines = []
    if args.es is not None:
        info_lines.append(f"es = {args.es}")
    if args.eps is not None:
        info_lines.append(f"eps = {fmt_eps(args.eps)}")
    info_lines.append(f"l = {final_cost(df):.4g}")

    style_axes(ax, subplot_title=title, info_lines=info_lines,
               has_collisions=n_collisions > 0)
    ax.grid(True, lw=0.4, alpha=0.6)
    ax.set_aspect("equal", adjustable="box")

    margin = 8.0  # metres
    ax.set_xlim(df["x"].min() - margin, df["x"].max() + margin)
    ax.set_ylim(df["y"].min() - margin, df["y"].max() + margin)

    fig.tight_layout()
    fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)

    print(f"plot saved -> {out}  ({n_collisions} collision point(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
