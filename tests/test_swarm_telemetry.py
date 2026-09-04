"""
Unit tests for Comprehensive Swarm Telemetry & Run Logging.
"""

import os
import json
import tempfile
import pytest
import numpy as np

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
from mars_swarm.swarm_telemetry import SwarmTelemetryLogger


def test_telemetry_energy_and_clearance_aggregation():
    """Verify energy integration and inter-robot clearance tracking."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        logger = SwarmTelemetryLogger(log_dir=tmp_dir, enable_tensorboard=False, dt=0.1)
        
        # 10 steps of motion
        for s in range(1, 11):
            actions = {
                'tb1': np.array([0.2, 0.0], dtype=np.float32),   # v=0.2, w=0.0 -> v^2=0.04 * 0.1 = 0.004J
                'tb2': np.array([0.0, 0.5], dtype=np.float32)    # v=0.0, w=0.5 -> w^2=0.25 * 0.1 = 0.025J
            }
            poses = {
                'tb1': (0.0, 0.0, 0.0),
                'tb2': (1.5, 0.0, 0.0) # distance = 1.5m
            }
            logger.record_step(s, acr=s * 5.0, actions_dict=actions, poses_dict=poses)
            
        results = {'distance': 2.0, 'redundancy': 1.0, 'acr': 50.0}
        summary = logger.finalize_and_export(results)
        
        # Total energy = 10 * (0.004 + 0.025) = 0.29J
        assert np.isclose(summary['cumulative_energy_joules_proxy'], 0.29, atol=1e-2)
        assert summary['normalized_energy_j_per_m'] > 0.0
        assert summary['inter_robot_clearance']['min_meters'] == 1.5
        assert len(summary['inter_robot_clearance']['histogram_counts']) == 10


def test_telemetry_mtbd_calculation():
    """Verify Mean Time Between Deadlocks (MTBD) calculation."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        logger = SwarmTelemetryLogger(log_dir=tmp_dir, enable_tensorboard=False)
        
        # 100 steps with deadlock events at step 20, 50, and 80 (intervals = 30 steps)
        for s in range(1, 101):
            deadlock = (s in [20, 50, 80])
            actions = {'tb1': np.array([0.1, 0.0])}
            poses = {'tb1': (0.0, 0.0, 0.0)}
            logger.record_step(s, acr=40.0, actions_dict=actions, poses_dict=poses, is_deadlock_event=deadlock)
            
        results = {'distance': 10.0, 'redundancy': 1.0, 'acr': 40.0}
        summary = logger.finalize_and_export(results)
        
        assert summary['deadlock_count'] == 3
        # Intervals are [30, 30] -> MTBD = 30.0 steps
        assert summary['mean_time_between_deadlocks_steps'] == 30.0


def test_telemetry_run_summary_json_export():
    """Verify run_summary.json is exported with proper schema."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        json_target = os.path.join(tmp_dir, "test_summary.json")
        logger = SwarmTelemetryLogger(log_dir=tmp_dir, enable_tensorboard=False)
        
        actions = {'tb1': np.array([0.1, 0.1])}
        poses = {'tb1': (0.0, 0.0, 0.0)}
        logger.record_step(1, acr=10.0, actions_dict=actions, poses_dict=poses)
        
        results = {'distance': 5.0, 'redundancy': 1.1, 'acr': 10.0}
        logger.finalize_and_export(results, export_path=json_target)
        
        assert os.path.exists(json_target)
        with open(json_target, 'r') as f:
            data = json.load(f)
            
        required_keys = [
            'final_acr_percent', 'total_steps', 'total_distance_meters',
            'cumulative_energy_joules_proxy', 'normalized_energy_j_per_m',
            'cell_overlap_redundancy', 'total_collisions', 'wall_collisions', 'agent_collisions',
            'deadlock_count', 'mean_time_between_deadlocks_steps', 'inter_robot_clearance', 'acr_curve_sampled'
        ]
        for k in required_keys:
            assert k in data, f"Missing required telemetry key: {k}"
