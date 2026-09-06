"""
Unit tests for Coverage Heatmap Renderer.
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from coverage_heatmap_renderer import render_coverage_heatmap, load_and_render, generate_demo_heatmap


def test_render_coverage_heatmap_output(tmp_path):
    """Verify render_coverage_heatmap generates a non-empty PNG file with trajectories and metrics."""
    h, w = 30, 40
    visited = np.zeros((h, w), dtype=bool)
    visited[10:20, 15:25] = True

    obstacles = np.zeros((h, w), dtype=bool)
    obstacles[0, :] = True
    obstacles[-1, :] = True
    obstacles[:, 0] = True
    obstacles[:, -1] = True

    trajectories = {
        'tb1': [(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)],
        'tb2': [(0.0, -0.7), (0.5, -1.2), (1.0, -2.0)]
    }

    out_file = os.path.join(tmp_path, "test_heatmap.png")
    result = render_coverage_heatmap(
        visited_grid=visited,
        obstacle_grid=obstacles,
        trajectories=trajectories,
        world_name="depot",
        grid_bounds=(-10.0, 10.0, -10.0, 10.0),
        acr_percent=35.5,
        steps=200,
        output_path=out_file
    )

    assert result == out_file
    assert os.path.exists(out_file)
    assert os.path.getsize(out_file) > 10000, "Generated heatmap PNG is too small or corrupted"
    assert os.path.exists(os.path.join(tmp_path, "test_heatmap.npz")), "Companion .npz was not generated"


def test_load_and_render_from_npz(tmp_path):
    """Verify load_and_render can reload saved .npz data and produce equivalent PNG."""
    npz_file = os.path.join(tmp_path, "run_data.npz")
    grid = np.zeros((20, 30), dtype=int)
    grid[5:15, 5:15] = 3
    np.savez_compressed(
        npz_file,
        visited_grid=grid,
        world_name="cafe",
        grid_bounds=np.array([-8.0, 8.0, -4.0, 4.0], dtype=np.float32),
        acr_percent=42.0,
        steps=150
    )

    out_png = os.path.join(tmp_path, "reloaded.png")
    res = load_and_render(npz_file, output_path=out_png, density_mode=True)
    assert res == out_png
    assert os.path.exists(out_png)
    assert os.path.getsize(out_png) > 10000


def test_generate_demo_heatmap(tmp_path):
    """Verify generate_demo_heatmap produces valid visualization for any world."""
    out_png = os.path.join(tmp_path, "demo_maze.png")
    res = generate_demo_heatmap(world_name="maze", output_path=out_png, steps=50, acr_target=12.5)
    assert res == out_png
    assert os.path.exists(out_png)
    assert os.path.getsize(out_png) > 10000
