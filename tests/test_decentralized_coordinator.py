"""
Unit tests for Decentralized Swarm Coordination (CBAA & Dynamic Voronoi Partitioning).
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
from mars_swarm.decentralized_coordinator import DecentralizedCoordinator


def test_voronoi_partitioning_disjoint_targets():
    """Verify in-range teammates partition candidate frontiers without duplicate claims."""
    coord = DecentralizedCoordinator(d_comm=3.0)
    
    poses = {
        'tb1': (0.0, 0.0, 0.0),
        'tb2': (2.0, 0.0, 0.0),  # In communication range (2.0m <= 3.0m)
    }
    
    grid_bounds = (-5.0, 5.0, -5.0, 5.0)
    local_visited = np.zeros((10, 10), dtype=bool)
    local_obs = np.zeros((10, 10), dtype=bool)
    
    # tb1 selects target
    target1, waypoint1, bid1 = coord.select_decentralized_frontier(
        'tb1', 1, poses, local_visited, local_obs, grid_bounds, 10, 10, set()
    )
    assert target1 is not None
    
    # tb2 selects target
    target2, waypoint2, bid2 = coord.select_decentralized_frontier(
        'tb2', 1, poses, local_visited, local_obs, grid_bounds, 10, 10, set()
    )
    assert target2 is not None
    
    # In-range communication must prevent duplicate claims
    assert target1 != target2, "Teammates in communication range selected the same target!"


def test_claim_expiration_on_peer_dropout():
    """Verify that if a robot drops out or crashes, its target claim expires after timeout."""
    coord = DecentralizedCoordinator(d_comm=3.0, claim_timeout_steps=10)
    
    poses = {
        'tb1': (0.0, 0.0, 0.0),
        'tb2': (1.0, 0.0, 0.0),
    }
    
    # tb2 claims cell (5, 5) at step 1
    coord.exchange_broadcasts(1, poses, 'tb2', (5, 5), 1.0)
    assert 'tb2' in coord.claims_table
    assert coord.claims_table['tb2'].target_cell == (5, 5)
    
    # tb2 drops out (no longer in poses)
    del poses['tb2']
    
    # At step 5, claim is still fresh (< 10 steps timeout)
    coord.exchange_broadcasts(5, poses, 'tb1', (2, 2), 0.5)
    assert 'tb2' in coord.claims_table
    
    # At step 15, timeout has elapsed (> 10 steps since step 1) -> claim expires
    coord.exchange_broadcasts(15, poses, 'tb1', (2, 2), 0.5)
    assert 'tb2' not in coord.claims_table, "Dead peer claim failed to expire after timeout!"
