"""
Unit tests for multi-world initialization, geometry bounds, and goal allocations.
Validates cafe, warehouse, depot, office, and maze without launching Gazebo.
"""

import os
import sys
import pytest
import numpy as np

# Add mars_swarm to PYTHONPATH
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))

from mars_swarm.multi_env_wrapper import PettingZooSwarmEnv


@pytest.mark.parametrize("world", ['cafe', 'warehouse', 'depot', 'office', 'maze'])
def test_world_initialization_and_geometry(world):
    """Verify that PettingZooSwarmEnv initializes with valid bounds and cell size ~0.4m for all worlds."""
    env = PettingZooSwarmEnv(world=world)
    
    # Check grid bounds: (min_x, max_x, min_y, max_y)
    min_x, max_x, min_y, max_y = env.grid_bounds
    assert max_x > min_x, f"Invalid X bounds for world {world}: {min_x} to {max_x}"
    assert max_y > min_y, f"Invalid Y bounds for world {world}: {min_y} to {max_y}"
    
    # Check resolutions
    assert env.grid_resolution_x > 0
    assert env.grid_resolution_y > 0
    
    # Verify cell size is approximately ~0.4m (tolerating 0.35m to 0.45m)
    cell_size_x = (max_x - min_x) / env.grid_resolution_x
    cell_size_y = (max_y - min_y) / env.grid_resolution_y
    assert 0.35 <= cell_size_x <= 0.45, f"Cell size X ({cell_size_x:.3f}m) out of range for world {world}"
    assert 0.35 <= cell_size_y <= 0.45, f"Cell size Y ({cell_size_y:.3f}m) out of range for world {world}"
    
    # Verify safe goals exist and are non-empty
    assert len(env.safe_goals) > 0, f"Safe goals empty for world {world}"
    assert len(env.safe_goals_world) == len(env.safe_goals)
    
    # Check visited grid shape matches resolution
    assert env.visited_grid.shape == (env.grid_resolution_y, env.grid_resolution_x)


def test_sdf_files_exist_for_all_worlds():
    """Verify that all declared worlds have corresponding SDF files in src/mars_swarm/worlds/."""
    worlds = ['cafe', 'warehouse', 'depot', 'office', 'maze']
    worlds_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'worlds'))
    
    for w in worlds:
        sdf_path = os.path.join(worlds_dir, f"{w}.sdf")
        assert os.path.isfile(sdf_path), f"Missing SDF file for world: {w} at {sdf_path}"
        with open(sdf_path, 'r') as f:
            content = f.read()
            assert "<world name=\"default\">" in content, f"SDF file for {w} missing default world tag"
            assert "<include>" in content, f"SDF file for {w} missing Fuel <include> tag"
