from pathlib import Path
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib import font_manager
from typing import Optional
import numpy as np
from matplotlib.patches import Rectangle

# === 1) IEEE-like font family and sizes (10pt doc) ===
S = {
    "normalsize": 10,      # body text
    "small": 9,            # axis labels / lane labels
    "footnotesize": 8,     # tick labels / legend
    "large": 12,           # figure title
}
mpl.rcParams.update({
    "text.usetex": False,
    "mathtext.fontset": "stix",                   # Times-like math
    "axes.titlesize": S["large"],
    "axes.labelsize": S["small"],
    "xtick.labelsize": S["footnotesize"],
    "ytick.labelsize": S["footnotesize"],
    "legend.fontsize": S["footnotesize"],
    "pdf.fonttype": 42, "ps.fonttype": 42,        # keep text searchable
})
# Pick an available Times-like font so figures stay consistent on systems
# without proprietary Times faces.
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
# ==========================================
CONFIGS = [
    ("_32_withoutQuanti", "Without Nondimensionalization"),
    ("_32_withQuanti", "With Nondimensionalization"),
]


def load_intersection_data(suffix: str) -> pd.DataFrame:
    """Return the intersection trajectory dataframe for the requested suffix."""
    repo_root = Path(__file__).resolve().parent
    build_dir = repo_root / "build"
    candidates = [
        Path.cwd() / f"trajectories_intersection{suffix}.csv",
        build_dir / f"trajectories_intersection{suffix}.csv",
    ]

    for path in candidates:
        if path.exists():
            return pd.read_csv(path)

    raise FileNotFoundError(
        f"Could not find trajectories file for suffix {suffix}. Searched: {', '.join(str(p) for p in candidates)}"
    )

# Function to plot trajectories
def plot_trajectories(df, scenario_name, ax):
    vehicle_ids = df["vehicle_id"].unique()
    
    for vid in vehicle_ids:
        vehicle_data = df[df["vehicle_id"] == vid]

        # Convert to numpy arrays
        x_vals = vehicle_data["x"].to_numpy()
        y_vals = vehicle_data["y"].to_numpy()
        psi_vals = vehicle_data["psi"].to_numpy()  # Vehicle heading (in radians)

        # Plot trajectory
        ax.plot(x_vals, y_vals, marker="o", label=f"Vehicle {vid}")

        # Draw the vehicle at the FIRST trajectory point (beginning)
        if len(x_vals) > 0:
            x_start, y_start, psi_start = x_vals[0], y_vals[0], psi_vals[0]

            # Vehicle dimensions
            vehicle_length = 4.5  # meters
            vehicle_width = 2.0  # meters

            # Compute bottom-left corner, considering both length & width
            x_corner = x_start - (vehicle_length / 2) * np.cos(psi_start) + (vehicle_width / 2) * np.sin(psi_start)
            y_corner = y_start - (vehicle_length / 2) * np.sin(psi_start) - (vehicle_width / 2) * np.cos(psi_start)

            # Create and add a rotated rectangle (vehicle shape)
            rect = Rectangle((x_corner, y_corner), vehicle_length, vehicle_width,
                             angle=np.degrees(psi_start), edgecolor='red', facecolor='none', lw=2)
            
            ax.add_patch(rect)  # Add vehicle shape to plot


def style_axes(
    ax,
    legend_title: Optional[str] = None,
    subplot_title: Optional[str] = None,
    show_legend: bool = True,
    show_xlabel: bool = True,
    show_ylabel: bool = True,
):
    axis_font = font_prop("large")
    title_font = font_prop("large")
    tick_font = font_prop("normalsize")

    if show_xlabel:
        ax.set_xlabel("X Position [m]", fontproperties=axis_font)
    if show_ylabel:
        ax.set_ylabel("Y Position [m]", fontproperties=axis_font)
    if subplot_title:
        ax.set_title(subplot_title, fontproperties=title_font)

    if show_legend:
        legend = ax.legend(title=legend_title)
        if legend:
            if legend_title:
                legend.get_title().set_fontproperties(tick_font)
            for text in legend.get_texts():
                text.set_fontproperties(tick_font)

    for tick in ax.get_xticklabels():
        tick.set_fontproperties(tick_font)
    for tick in ax.get_yticklabels():
        tick.set_fontproperties(tick_font)

# Create subplots for each configuration (side by side)
fig, axes = plt.subplots(
    1,
    len(CONFIGS),
    figsize=(6* len(CONFIGS), 6),
    sharex=True,
    sharey=True,
    gridspec_kw={"wspace": 0.05},
)
axes = np.atleast_1d(axes).ravel()

for idx, (ax, (suffix, title)) in enumerate(zip(axes, CONFIGS)):
    df_intersection = load_intersection_data(suffix)
    plot_trajectories(df_intersection, "Intersection", ax)
    style_axes(
        ax,
        legend_title=None,
        show_legend=False,
        show_xlabel=False,
        show_ylabel=False,
        subplot_title=title,
    )
    ax.grid(True)
    ax.set_aspect("equal", adjustable="box")  # Keep proportions realistic without conflicting with shared axes
    if idx > 0:
        ax.tick_params(labelleft=False)

# Add shared axis labels
fig.text(0.5, 0.02, "X Position [m]", fontproperties=font_prop("large"), ha="center")
fig.text(0.02, 0.5, "Y Position [m]", fontproperties=font_prop("large"), va="center", rotation=90)

# Show the plot
fig.subplots_adjust(left=0.08, right=0.98, top=0.97, bottom=0.12, wspace=0.05)
plt.show()
