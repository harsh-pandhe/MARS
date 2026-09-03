"""Unit tests for A* pathfinding, obstacle inflation, and frontier targeting."""

import pytest
import numpy as np
import sys
import os

# Include mars_swarm module
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from evaluate_benchmarks import astar_path, inflate_obstacles, line_of_sight_clear


def test_astar_straight_line():
    """A* finds direct path in an open grid."""
    grid = np.zeros((10, 10), dtype=bool)
    path = astar_path(grid, (0, 0), (5, 5))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (5, 5)


def test_astar_obstacle_detour():
    """A* routes around a wall blocking direct line of sight."""
    grid = np.zeros((10, 10), dtype=bool)
    # Horizontal wall across row 3, columns 0..7
    grid[3, 0:8] = True
    
    # Path from (0, 0) to (5, 0) must route around the wall opening at col 8 or 9
    path = astar_path(grid, (0, 0), (5, 0))
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (5, 0)
    # Check that no waypoint is inside the wall
    for r, c in path:
        assert not grid[r, c]


def test_astar_unreachable_target():
    """A* returns None when target is completely surrounded by obstacles."""
    grid = np.zeros((10, 10), dtype=bool)
    # Enclose cell (5, 5)
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if (dr, dc) != (0, 0):
                grid[5 + dr, 5 + dc] = True
                
    path = astar_path(grid, (0, 0), (5, 5))
    assert path is None


def test_obstacle_inflation():
    """Dilation inflates obstacle radius by configured cells."""
    grid = np.zeros((7, 7), dtype=bool)
    grid[3, 3] = True
    
    inflated = inflate_obstacles(grid, radius=1)
    # The 3x3 block around (3,3) should all be True
    assert np.all(inflated[2:5, 2:5] == True)
    # Corners should be False
    assert inflated[0, 0] == False
    assert inflated[6, 6] == False


def test_line_of_sight():
    """Line-of-sight checks unobstructed vs obstructed paths."""
    grid = np.zeros((10, 10), dtype=bool)
    assert line_of_sight_clear(grid, (1, 1), (8, 8)) is True
    
    grid[4, 4] = True
    assert line_of_sight_clear(grid, (1, 1), (8, 8)) is False
