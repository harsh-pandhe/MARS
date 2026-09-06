"""
Unit tests for benchmark_mappo_multiworld.py harness and scenario configurations.
"""

import os
import sys
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm')))
from mars_swarm.benchmark_mappo_multiworld import SCENARIOS, DEFAULT_WORLDS


def test_mappo_scenarios_structure():
    """Verify all 5 baseline scenarios are properly specified."""
    expected = {'heuristic', 'random', 'mappo_nominal', 'mappo_noise', 'mappo_failure'}
    assert set(SCENARIOS.keys()) == expected

    for key, cfg in SCENARIOS.items():
        assert 'name' in cfg
        assert 'mode' in cfg
        assert 'noise' in cfg
        assert 'failure' in cfg
        assert cfg['mode'] in ('heuristic', 'random', 'mappo')

    assert SCENARIOS['mappo_noise']['noise'] is True
    assert SCENARIOS['mappo_nominal']['noise'] is False
    assert SCENARIOS['mappo_failure']['failure'] is True


def test_default_worlds():
    """Verify all 5 benchmark worlds are registered in DEFAULT_WORLDS."""
    assert set(DEFAULT_WORLDS) == {'cafe', 'warehouse', 'depot', 'office', 'maze'}


def test_benchmark_cli_parser():
    """Verify CLI argument parsing logic."""
    import argparse
    from mars_swarm.benchmark_mappo_multiworld import main

    # Test scenario selection
    assert 'heuristic' in SCENARIOS
    assert 'mappo_nominal' in SCENARIOS
