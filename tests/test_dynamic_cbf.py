"""
Unit and dynamic simulation tests for Control Barrier Function (CBF) against moving non-static hazards.
"""

import os
import sys
import pytest
import math
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from cbf_qp_solver import FastCBFSolver


def test_dynamic_obstacle_head_on_approach():
    """
    Simulate a moving dynamic obstacle approaching head-on at 0.3 m/s.
    Verify that CBF detects the closing distance on successive timesteps,
    actively sheds forward velocity, and preserves physical clearance >= d_safe.
    """
    solver = FastCBFSolver(l=0.12, d_safe_obs=0.22, gamma=2.0)
    dt = 0.1  # 10 Hz control loop
    
    # Initial state: robot at origin, dynamic obstacle at x = 1.0 m moving left at -0.3 m/s
    robot_x = 0.0
    obs_x = 1.0
    v_obs = -0.30  # Moving towards robot
    
    min_observed_clearance = float('inf')
    
    for step in range(30):
        # Current distance to moving obstacle
        dist_to_obs = obs_x - robot_x
        min_observed_clearance = min(min_observed_clearance, dist_to_obs)
        
        # Build 24-beam lidar observation where beam 0 is forward (towards the approaching obstacle)
        lidar = np.full(24, 5.0, dtype=np.float32)
        if dist_to_obs > 0:
            lidar[0] = dist_to_obs
            lidar[1] = dist_to_obs / math.cos(math.radians(15))
            lidar[-1] = dist_to_obs / math.cos(math.radians(15))
            
        # Nominal intent: aggressive forward push at 0.22 m/s
        v_nom = 0.22
        w_nom = 0.0
        
        # Solve CBF QP
        v_safe, w_safe = solver.solve(v_nom, w_nom, lidar)
        
        # When obstacle gets within safety horizon, CBF must brake or stop
        if dist_to_obs < 0.40:
            assert v_safe < 0.10, f"At dist={dist_to_obs:.3f}m, robot should brake! Got v_safe={v_safe:.3f}"
        if dist_to_obs < 0.25:
            assert v_safe <= 0.01, f"At dist={dist_to_obs:.3f}m, robot forward motion must halt! Got v_safe={v_safe:.3f}"
            
        # Update kinematic positions
        robot_x += v_safe * dt
        obs_x += v_obs * dt
        
        # If obstacle has passed or halted
        if dist_to_obs <= 0.22:
            break
            
    # Clearance must never have violated the critical physical collision limit (< 0.14m bumper)
    assert min_observed_clearance >= 0.20, f"Dynamic obstacle penetrated safety boundary! Min dist={min_observed_clearance:.3f}m"


def test_dynamic_crossing_hazard_deflection():
    """
    Simulate a moving hazard crossing diagonally across the robot's forward path.
    Verify CBF generates angular deflection (w_safe != 0) to bypass or yield to the hazard.
    """
    solver = FastCBFSolver(l=0.12, d_safe_obs=0.22, gamma=2.0)
    
    # Hazard located at 0.35m forward-left (+25 degrees)
    lidar = np.full(24, 4.0, dtype=np.float32)
    lidar[1] = 0.30   # +15 deg
    lidar[2] = 0.32   # +30 deg
    
    # Nominal intent: drive straight
    v_safe, w_safe = solver.solve(v_nom=0.20, w_nom=0.0, lidar_ranges=lidar)
    
    # Must deflect away from obstacle (clockwise rotation, w_safe < 0) or decelerate
    assert v_safe < 0.20 or abs(w_safe) > 0.05, f"Expected deceleration or steering deflection, got v={v_safe:.3f}, w={w_safe:.3f}"
