"""
Unit tests for Coverage Heatmap Renderer.
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from coverage_heatmap_renderer import render_coverage_heatmap


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
