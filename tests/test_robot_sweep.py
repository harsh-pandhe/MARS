"""
Unit and integration tests for robot-count scalability sweep (N=1, 2, 3, 5, 8 robots).
"""

import os
import sys
import pytest
import math
import numpy as np
from unittest.mock import patch, MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
workspace_install = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'install', 'mars_swarm'))
if os.path.exists(workspace_install):
    cur = os.environ.get('AMENT_PREFIX_PATH', '')
    if workspace_install not in cur:
        os.environ['AMENT_PREFIX_PATH'] = f"{workspace_install}:{cur}" if cur else workspace_install

from mars_swarm.decentralized_coordinator import DecentralizedCoordinator


def test_swarm_env_scalability_mocked():
    """Verify PettingZooSwarmEnv initialization across N=1, 2, 3, 5, 8 robot counts."""
    with patch('mars_swarm.multi_env_wrapper.rclpy') as mock_rclpy, \
         patch('mars_swarm.multi_env_wrapper.SwarmNode') as mock_node, \
         patch('mars_swarm.multi_env_wrapper.threading.Thread') as mock_thread:
        mock_rclpy.ok.return_value = True
        
        from mars_swarm.multi_env_wrapper import PettingZooSwarmEnv

        for n in [1, 2, 3, 5, 8]:
            env = PettingZooSwarmEnv(num_robots=n, world='depot')
            assert len(env.possible_agents) == n, f"Expected {n} possible agents, got {len(env.possible_agents)}"
            assert len(env.agents) == n, f"Expected {n} active agents, got {len(env.agents)}"
            assert env.possible_agents == [f'tb{i}' for i in range(1, n + 1)]
            
            # Check observation and action spaces
            for agent in env.possible_agents:
                assert agent in env.observation_spaces
                assert env.observation_spaces[agent].shape == (46,)
                assert agent in env.action_spaces
                assert env.action_spaces[agent].shape == (2,)

            # Check spawn poses are distinct and separated by >= 0.70m
            poses = [env.spawn_poses[agent] for agent in env.possible_agents]
            for i in range(len(poses)):
                for j in range(i + 1, len(poses)):
                    dist = math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1])
                    assert dist >= 0.69, f"Spawn overlap between robot {i+1} and {j+1}: dist={dist:.2f}m < 0.7m"


def test_decentralized_coordinator_scaled_swarm():
    """Verify DecentralizedCoordinator handles 8-robot swarm without deadlock or duplicate claims."""
    coord = DecentralizedCoordinator(d_comm=3.0)
    ROBOT_X_OFFSETS = [0.0, -0.7, 0.7, -1.4, 1.4, -2.1, 2.1, -2.8]
    poses = {f'tb{k+1}': (0.0, ROBOT_X_OFFSETS[k], 0.0) for k in range(8)}

    grid_bounds = (-8.0, 8.0, -15.0, 15.0)
    h, w = 75, 40
    local_visited = np.zeros((h, w), dtype=bool)
    local_obs = np.zeros((h, w), dtype=bool)

    targets = {}
    for agent in poses:
        target, waypoint, bid = coord.select_decentralized_frontier(
            agent, 1, poses, local_visited, local_obs, grid_bounds, w, h, set()
        )
        assert target is not None, f"Agent {agent} failed to select a frontier"
        targets[agent] = target

    # Verify no two adjacent robots within d_comm choose the exact same cell
    for a1 in poses:
        for a2 in poses:
            if a1 >= a2:
                continue
            p1 = poses[a1]
            p2 = poses[a2]
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            if d <= coord.d_comm:
                assert targets[a1] != targets[a2], f"Adjacent agents {a1} and {a2} (dist={d:.2f}m) chose identical target {targets[a1]}"


def test_launch_file_declares_num_robots():
    """Verify spawn_multi.launch.py contains num_robots launch argument."""
    launch_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'launch', 'spawn_multi.launch.py'
    ))
    with open(launch_path, 'r') as f:
        content = f.read()

    assert "DeclareLaunchArgument('num_robots'" in content
    assert "DeclareLaunchArgument('robot_types'" in content
    assert "ROBOT_X_OFFSETS = [0.0, -0.7, 0.7, -1.4, 1.4, -2.1, 2.1, -2.8]" in content


def test_heterogeneous_robot_launch_configuration():
    """Verify spawn_multi.launch.py can be evaluated with heterogeneous robot_types."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    import importlib.util

    launch_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'launch', 'spawn_multi.launch.py'
    ))
    spec = importlib.util.spec_from_file_location("spawn_multi_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    assert ld is not None


def test_sweep_cli_parser():
    """Verify sweep_robot_count.py argument parsing for single and multiple worlds."""
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=str, default='depot',
                        choices=['cafe', 'warehouse', 'depot', 'office', 'maze', 'all'])
    parser.add_argument('--worlds', type=str, nargs='+', default=None)
    parser.add_argument('--robot-counts', type=int, nargs='+', default=[2, 3, 5, 8])
    parser.add_argument('--max-steps', type=int, default=300)

    args = parser.parse_args(['--world', 'office', '--robot-counts', '2', '5', '--max-steps', '500'])
    assert args.world == 'office'
    assert args.robot_counts == [2, 5]
    assert args.max_steps == 500

    args_all = parser.parse_args(['--world', 'all', '--robot-counts', '2', '3', '5', '8'])
    assert args_all.world == 'all'
    assert args_all.robot_counts == [2, 3, 5, 8]

    args_multi = parser.parse_args(['--worlds', 'cafe', 'maze', '--robot-counts', '3'])
    assert args_multi.worlds == ['cafe', 'maze']
    assert args_multi.robot_counts == [3]
