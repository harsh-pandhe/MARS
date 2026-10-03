"""Property tests for the neighbor encoder in TorchCentralizedCriticModel.

The observation is a fixed 46-vector: 28 ego features followed by 9 neighbor
slots of (distance, bearing); absent neighbors are padded with (10.0, 0.0) and
masked out (distance >= 9.0). The actor should therefore be invariant to the
order of neighbors and to where the padding sits. These tests check that
property directly; they do NOT test behaviour at a different swarm size, which
would need a trained policy.
"""
import os
import sys

import numpy as np
import pytest
import torch
from gymnasium.spaces import Box

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from train_multi import TorchCentralizedCriticModel  # noqa: E402


def _model():
    torch.manual_seed(0)
    m = TorchCentralizedCriticModel(
        Box(-np.inf, np.inf, (46,), np.float32), Box(-1.0, 1.0, (2,), np.float32),
        4, {}, "inv_test")
    m.eval()
    return m


def _obs(ego, neighbors):
    """neighbors: list of (dist, angle); padded to 9 slots with (10.0, 0.0)."""
    slots = list(neighbors) + [(10.0, 0.0)] * (9 - len(neighbors))
    return np.concatenate([ego, np.array(slots, dtype=np.float32).reshape(-1)]).astype(np.float32)


def _out(model, obs):
    with torch.no_grad():
        out, _ = model.forward({"obs": torch.as_tensor(obs[None, :])}, None, None)
    return out.numpy()[0]


def test_output_invariant_to_neighbor_order():
    rng = np.random.default_rng(1)
    m = _model()
    ego = rng.random(28).astype(np.float32)
    nbrs = [(float(rng.uniform(0.5, 5.0)), float(rng.uniform(-3, 3))) for _ in range(5)]
    base = _out(m, _obs(ego, nbrs))
    for _ in range(10):
        perm = rng.permutation(len(nbrs))
        np.testing.assert_allclose(_out(m, _obs(ego, [nbrs[i] for i in perm])), base, atol=1e-5)


def test_output_invariant_to_padding_position():
    rng = np.random.default_rng(2)
    m = _model()
    ego = rng.random(28).astype(np.float32)
    nbrs = [(2.0, 0.5), (3.5, -1.0), (1.2, 2.0)]
    base = _out(m, _obs(ego, nbrs))
    # place the same three neighbors in arbitrary slots among 9
    slots = [(10.0, 0.0)] * 9
    for dst, n in zip([7, 0, 4], nbrs):
        slots[dst] = n
    obs = np.concatenate([ego, np.array(slots, dtype=np.float32).reshape(-1)]).astype(np.float32)
    np.testing.assert_allclose(_out(m, obs), base, atol=1e-5)


def test_neighbors_actually_change_the_output():
    """Sanity check so the invariance tests above cannot pass vacuously."""
    rng = np.random.default_rng(3)
    m = _model()
    ego = rng.random(28).astype(np.float32)
    a = _out(m, _obs(ego, [(1.0, 0.3)]))
    b = _out(m, _obs(ego, [(4.0, -2.0)]))
    none = _out(m, _obs(ego, []))
    assert not np.allclose(a, b, atol=1e-4)
    assert not np.allclose(a, none, atol=1e-4)
