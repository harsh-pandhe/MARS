"""
Unit tests for Deterministic Seed Management and Single-Threaded Physics Stepping.
"""

import os
import re
import pytest


def test_cafe_sdf_has_single_threaded_ode():
    """Verify cafe.sdf contains single-threaded ODE physics configuration."""
    world_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'worlds', 'cafe.sdf'))
    with open(world_path, 'r') as f:
        content = f.read()
        
    assert '<physics name="1ms" type="ode">' in content, "Missing ODE physics block in cafe.sdf"
    assert '<thread_count>1</thread_count>' in content, "Missing single-threaded thread_count in cafe.sdf"


def test_warehouse_sdf_has_single_threaded_ode():
    """Verify warehouse.sdf contains single-threaded ODE physics configuration."""
    world_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'worlds', 'warehouse.sdf'))
    with open(world_path, 'r') as f:
        content = f.read()
        
    assert '<physics name="1ms" type="ode">' in content, "Missing ODE physics block in warehouse.sdf"
    assert '<thread_count>1</thread_count>' in content, "Missing single-threaded thread_count in warehouse.sdf"


def test_spawn_multi_declares_seed_argument():
    """Verify spawn_multi.launch.py declares seed argument and passes --seed to gz_args."""
    launch_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'launch', 'spawn_multi.launch.py'))
    with open(launch_path, 'r') as f:
        content = f.read()
        
    assert "DeclareLaunchArgument('seed'" in content, "Missing seed DeclareLaunchArgument in spawn_multi.launch.py"
    assert "--seed" in content, "Missing --seed flag in gz_args in spawn_multi.launch.py"
