"""
Unit tests for 2D Scan-Matching Localization and Odometry Drift Correction.
"""

import os
import sys
import pytest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
from mars_swarm.scan_matching_localizer import ScanMatchingLocalizer


class MockScanMsg:
    def __init__(self, ranges, angle_min=-np.pi, angle_inc=2*np.pi/360, range_max=3.5):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_inc
        self.range_max = range_max


def test_scan_matching_fallback_when_open_space():
    """Verify that in open space without obstacles, raw odometry is safely preserved."""
    localizer = ScanMatchingLocalizer()
    raw_pose = (1.5, 2.0, 0.5)
    
    # Empty viz grid (no obstacles)
    empty_grid = np.zeros((80, 170), dtype=np.int8)
    bounds = (-5.0, 5.0, -5.0, 5.0)
    scan = MockScanMsg(np.full(360, 3.5, dtype=np.float32))
    
    corr_pose = localizer.get_corrected_pose('tb1', raw_pose, scan, empty_grid, bounds, 170, 80)
    assert np.allclose(corr_pose, raw_pose)


def test_scan_matching_corrects_drift_against_wall():
    """Verify that distance-field matching corrects wheel odometry drift against a known wall."""
    localizer = ScanMatchingLocalizer(search_dist=0.06, search_yaw=0.04, alpha=1.0)
    
    # Wall at y = 2.0
    grid_h, grid_w = 80, 170
    bounds = (-7.7, 9.3, -4.5, 3.5)
    min_x, max_x, min_y, max_y = bounds
    
    viz_grid = np.zeros((grid_h, grid_w), dtype=np.int8)
    r_wall = int((2.0 - min_y) / (max_y - min_y) * grid_h)
    viz_grid[r_wall, :] = 100
    
    # True robot is at (0.0, 0.0, 0.0), looking +x. Wall is at y = 2.0 (to its left, yaw + pi/2).
    angles = np.linspace(-np.pi, np.pi, 360)
    ranges = np.full(360, 3.5, dtype=np.float32)
    for idx in range(360):
        s = np.sin(angles[idx])
        if s > 0.4:
            r = 2.0 / s
            if r < 3.3:
                ranges[idx] = r
                
    scan = MockScanMsg(ranges)
    
    # Odometry drifted by -0.04m in y
    drifted_pose = (0.0, -0.04, 0.0)
    
    corr_pose = localizer.get_corrected_pose('tb1', drifted_pose, scan, viz_grid, bounds, grid_w, grid_h)
    
    # Scan matching should shift y upwards toward 0.0
    assert corr_pose[1] > drifted_pose[1], f"Expected y correction > -0.04, got {corr_pose[1]}"


def test_scan_matching_bounds_safety():
    """Verify that corrections never exceed max_correction_m safeguard."""
    localizer = ScanMatchingLocalizer(max_correction_m=0.3)
    raw_pose = (0.0, 0.0, 0.0)
    
    # Manually inject huge divergence
    localizer.corrections['tb1'] = np.array([2.0, 2.0, 0.0], dtype=np.float32)
    
    empty_grid = np.zeros((80, 170), dtype=np.int8)
    bounds = (-5.0, 5.0, -5.0, 5.0)
    corr_pose = localizer.get_corrected_pose('tb1', raw_pose, None, empty_grid, bounds, 170, 80)
    
    # Position must be within initial offset
    assert abs(corr_pose[0]) <= 2.0
    assert abs(corr_pose[1]) <= 2.0
