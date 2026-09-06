"""
Coverage Heatmap Renderer.

Transforms the swarm's visited grid, belief occupancy grid, and robot trajectories
into high-resolution, publication-grade visual coverage heatmaps and PNGs.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.patches as patches


def render_coverage_heatmap(
    visited_grid,
    obstacle_grid=None,
    trajectories=None,
    world_name="cafe",
    grid_bounds=(-15.0, 15.0, -25.0, 25.0),
    acr_percent=0.0,
    steps=0,
    output_path="docs/heatmaps/coverage_heatmap.png",
    dpi=300
):
    """
    Renders a publication-ready coverage heatmap.

    Args:
        visited_grid (2D ndarray): Boolean or integer visit-count array (H, W).
        obstacle_grid (2D ndarray, optional): Boolean array of static obstacle cells (H, W).
        trajectories (dict, optional): Map of agent_id -> list of (x, y) coordinates in odom frame.
        world_name (str): Name of the world environment ('cafe', 'warehouse', 'depot', etc.).
        grid_bounds (tuple): (min_x, max_x, min_y, max_y) in local odom coordinates.
        acr_percent (float): Final Area Coverage Rate %.
        steps (int): Total simulation steps elapsed.
        output_path (str): Target image filepath (.png).
        dpi (int): Output figure resolution.

    Returns:
        output_path (str): Filepath of the generated heatmap.
    """
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    h, w = visited_grid.shape
    min_x, max_x, min_y, max_y = grid_bounds

    # Create figure with dark modern styling
    fig, ax = plt.subplots(figsize=(10, 8), facecolor='#0f172a')
    ax.set_facecolor('#0f172a')

    # Prepare display matrix:
    # 0 = Unexplored (navy/charcoal)
    # 1 = Visited (teal to bright amber glow)
    # -1 = Obstacle / Wall (dark graphite)
    display_layer = np.zeros((h, w), dtype=np.float32)
    display_layer[visited_grid > 0] = 1.0

    if obstacle_grid is not None and obstacle_grid.shape == (h, w):
        display_layer[obstacle_grid] = -1.0

    # Custom colormap:
    # -1.0 -> #334155 (Slate 700 - Obstacles)
    # 0.0  -> #1e293b (Slate 800 - Unexplored floor)
    # 1.0  -> #10b981 / #38bdf8 (Emerald / Cyan glow - Explored)
    cmap = LinearSegmentedColormap.from_list(
        'mars_coverage',
        [
            (0.0, '#334155'),   # -1: Obstacles
            (0.5, '#1e293b'),   # 0: Unexplored
            (0.75, '#0284c7'),  # Low-density explored
            (1.0, '#10b981'),   # Visited
        ]
    )

    norm_layer = (display_layer + 1.0) / 2.0  # map [-1, 1] -> [0, 1]
    extent = [min_x, max_x, min_y, max_y]

    im = ax.imshow(
        norm_layer,
        cmap=cmap,
        origin='lower',
        extent=extent,
        aspect='equal',
        interpolation='nearest'
    )

    # Plot robot trajectory overlays
    agent_colors = {
        'tb1': '#38bdf8',  # sky blue
        'tb2': '#f43f5e',  # rose
        'tb3': '#fbbf24',  # amber
        'tb4': '#a855f7',  # purple
        'tb5': '#34d399',  # emerald
        'tb6': '#fb923c',  # orange
        'tb7': '#ec4899',  # pink
        'tb8': '#60a5fa',  # light blue
    }

    if trajectories:
        for agent_id, points in trajectories.items():
            if len(points) < 2:
                continue
            color = agent_colors.get(agent_id, '#e2e8f0')
            pts = np.array(points)
            ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=1.5, alpha=0.85, label=f"{agent_id} path")
            # Mark starting pose
            ax.plot(pts[0, 0], pts[0, 1], marker='o', markersize=6, color=color, markeredgecolor='white', markeredgewidth=1.0)
            # Mark final pose
            ax.plot(pts[-1, 0], pts[-1, 1], marker='^', markersize=7, color=color, markeredgecolor='white', markeredgewidth=1.0)

    # Clean axes formatting
    ax.tick_params(colors='#94a3b8', labelsize=9)
    ax.set_xlabel("Local X (m)", color='#cbd5e1', fontsize=11, labelpad=8)
    ax.set_ylabel("Local Y (m)", color='#cbd5e1', fontsize=11, labelpad=8)
    for spine in ax.spines.values():
        spine.set_color('#334155')

    # Add subtitle & metrics banner
    title = f"MARS Swarm Coverage Map — {world_name.upper()}"
    ax.set_title(title, color='#f8fafc', fontsize=14, fontweight='bold', pad=14)

    metrics_text = f"Coverage (ACR): {acr_percent:.1f}%  |  Steps: {steps}  |  Grid: {w}x{h}"
    fig.text(0.5, 0.02, metrics_text, ha='center', color='#94a3b8', fontsize=10, family='monospace')

    if trajectories:
        leg = ax.legend(loc='upper right', facecolor='#1e293b', edgecolor='#475569', labelcolor='#f1f5f9', fontsize=8)

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    plt.savefig(output_path, dpi=dpi, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)

    print(f"[heatmap] Coverage heatmap generated: {output_path}")
    return output_path
