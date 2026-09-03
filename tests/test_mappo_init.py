"""Unit and regression tests for MAPPO policy, Centralized Critic, and Adam optimizer under Ray RLlib."""

import os
os.environ["TORCHDYNAMO_DISABLE"] = "1"
os.environ["TORCH_COMPILE_DISABLE"] = "1"

import sys
import pytest
import numpy as np
import torch
if hasattr(torch, "_dynamo"):
    torch._dynamo.config.disable = True
    torch._dynamo.config.suppress_errors = True

import ray
import gymnasium as gym
from ray.tune.registry import register_env
from ray.rllib.models import ModelCatalog
from ray.rllib.env.multi_agent_env import MultiAgentEnv

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm', 'mars_swarm')))
from train_multi import TorchCentralizedCriticModel, CentralizedCritic, PPOConfig


def test_standalone_adam_init():
    """Verify PyTorch Adam optimizer instantiates with parameters without segfault."""
    linear = torch.nn.Linear(46, 64)
    optimizer = torch.optim.Adam(linear.parameters(), lr=1e-4)
    assert optimizer is not None
    assert len(optimizer.param_groups) == 1


def test_mappo_rllib_training_step():
    """Verify CentralizedCritic builds and runs an optimization step under Ray/RLlib."""
    ModelCatalog.register_custom_model("cc_model_test", TorchCentralizedCriticModel)
    obs_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(46,), dtype=np.float32)
    act_space = gym.spaces.Box(low=np.array([-0.22, -1.0]), high=np.array([0.22, 1.0]), dtype=np.float32)

    if not ray.is_initialized():
        ray.init(ignore_reinit_error=True)

    class MockSwarmEnv(MultiAgentEnv):
        def __init__(self, config=None):
            super().__init__()
            self._agent_ids = {'tb1', 'tb2', 'tb3'}
            self.observation_spaces = {a: obs_space for a in self._agent_ids}
            self.action_spaces = {a: act_space for a in self._agent_ids}
            self._steps = 0
        def reset(self, *, seed=None, options=None):
            self._steps = 0
            obs = {a: np.random.randn(46).astype(np.float32) for a in self._agent_ids}
            infos = {a: {} for a in self._agent_ids}
            return obs, infos
        def step(self, action_dict):
            self._steps += 1
            obs = {a: np.random.randn(46).astype(np.float32) for a in self._agent_ids}
            rewards = {a: float(np.random.randn()) for a in self._agent_ids}
            terminateds = {a: (self._steps >= 10) for a in self._agent_ids}
            terminateds['__all__'] = (self._steps >= 10)
            truncateds = {'__all__': False}
            infos = {a: {} for a in self._agent_ids}
            return obs, rewards, terminateds, truncateds, infos

    register_env("mock_swarm_test_v0", lambda cfg: MockSwarmEnv(cfg))

    config = (
        PPOConfig()
        .api_stack(
            enable_env_runner_and_connector_v2=False,
            enable_rl_module_and_learner=False,
        )
        .environment("mock_swarm_test_v0")
        .framework("torch")
        .env_runners(num_env_runners=0, rollout_fragment_length=10)
        .training(
            model={"custom_model": "cc_model_test"},
            train_batch_size=30,
            minibatch_size=16,
            num_epochs=1,
            lr=1e-4,
        )
        .multi_agent(
            policies={"shared_policy": (None, obs_space, act_space, {})},
            policy_mapping_fn=lambda agent_id, *args, **kwargs: "shared_policy",
        )
        .resources(num_gpus=0)
    )

    algo = CentralizedCritic(config=config)
    result = algo.train()
    assert "env_runners" in result or "episode_reward_mean" in result
    algo.stop()
    ray.shutdown()
