"""Unit and regression tests for Area Coverage Rate (ACR) fairness."""

import pytest
import numpy as np


def compute_coverage_uniform(visited_grid, obstacle_grid):
    """Compute reachable coverage percentage excluding obstacle cells uniformly."""
    total_cells = visited_grid.size
    obstacle_cells = np.count_nonzero(obstacle_grid)
    reachable_cells = total_cells - obstacle_cells
    
    if reachable_cells <= 0:
        return 0.0
        
    # Valid visited cells cannot include obstacles
    valid_visited = np.logical_and(visited_grid, np.logical_not(obstacle_grid))
    covered_count = np.count_nonzero(valid_visited)
    
    return (covered_count / reachable_cells) * 100.0


def test_acr_fairness_empty_world():
    """In an open world with no obstacles, all visited cells count toward 100% reachable."""
    visited = np.zeros((10, 10), dtype=bool)
    obstacles = np.zeros((10, 10), dtype=bool)
    
    visited[0:5, :] = True  # 50 cells visited out of 100
    cov = compute_coverage_uniform(visited, obstacles)
    assert np.isclose(cov, 50.0)


def test_acr_fairness_excludes_obstacle_cells():
    """In a world with 20% obstacles, visiting all 80 reachable cells yields 100% coverage."""
    visited = np.ones((10, 10), dtype=bool)
    obstacles = np.zeros((10, 10), dtype=bool)
    obstacles[0:2, :] = True  # 20 obstacle cells
    
    # Even if visited contains obstacle cells (e.g. sensor overlap), they are excluded
    cov = compute_coverage_uniform(visited, obstacles)
    assert np.isclose(cov, 100.0)


def test_acr_fairness_partial_coverage_with_obstacles():
    """Reachable denominator is always (total - obstacles)."""
    visited = np.zeros((10, 10), dtype=bool)
    obstacles = np.zeros((10, 10), dtype=bool)
    obstacles[0:2, :] = True  # 20 obstacles -> 80 reachable
    
    visited[2:6, :] = True   # 40 reachable cells visited
    cov = compute_coverage_uniform(visited, obstacles)
    assert np.isclose(cov, 50.0)  # 40 / 80 = 50%
