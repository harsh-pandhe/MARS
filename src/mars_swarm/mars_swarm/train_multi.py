import os
import sys
import time
import signal
import subprocess
import argparse
import numpy as np
import torch
import torch.nn as nn

# Ensure path includes workspace packages
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from multi_env_wrapper import PettingZooSwarmEnv

from ray.rllib.models.torch.torch_modelv2 import TorchModelV2
from ray.rllib.models.torch.fcnet import FullyConnectedNetwork as TorchFC
from ray.rllib.models.torch.misc import SlimFC
from ray.rllib.utils.annotations import override
from ray.rllib.evaluation.postprocessing import Postprocessing, compute_advantages
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.utils.torch_utils import convert_to_torch_tensor
from ray.rllib.models import ModelCatalog
from ray.rllib.algorithms.ppo.ppo import PPO, PPOConfig
from ray.rllib.algorithms.ppo.ppo_torch_policy import PPOTorchPolicy

# --- Gazebo Launcher Helpers ---
gazebo_process = None

def start_gazebo(headless=True, multi=True):
    global gazebo_process
    print(f"[train_multi] Starting {'Multi-Robot' if multi else 'Single-Robot'} Gazebo simulation in background...")
    cmd = [
        "ros2", "launch", "mars_swarm", "spawn_multi.launch.py",
        f"headless:={str(headless).lower()}",
        f"multi:={str(multi).lower()}"
    ]
    # Launch in a separate process group so we can terminate the entire tree
    gazebo_process = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid
    )
    print("[train_multi] Gazebo process launched. Waiting for topics...")
    time.sleep(12.0)  # Wait for Gazebo and ROS 2 bridges to fully start up

    if not headless:
        from ament_index_python.packages import get_package_share_directory
        try:
            rviz_config_path = os.path.join(
                get_package_share_directory('mars_swarm'),
                'rviz', 'namespaced_swarm.rviz'
            )
            print(f"[train_multi] Starting RViz2 with config: {rviz_config_path}")
            subprocess.Popen(
                ["rviz2", "-d", rviz_config_path, "--ros-args", "-p", "use_sim_time:=true"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"[train_multi] [WARNING] Failed to start RViz2: {e}")

def kill_stale_processes():
    import signal
    current_pid = os.getpid()
    for name in os.listdir('/proc'):
        if name.isdigit():
            pid = int(name)
            if pid == current_pid:
                continue
            try:
                with open(os.path.join('/proc', name, 'cmdline'), 'rb') as f:
                    cmdline = f.read().decode('utf-8', errors='ignore').replace('\x00', ' ')
                lower_cmd = cmdline.lower()
                target_terms = ['gz sim', 'parameter_bridge', 'ros_gz_bridge', 'spawn_multi.launch.py', 'spawn_tb3.launch.py', 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'rviz2']
                if any(term in lower_cmd for term in target_terms):
                    os.kill(pid, signal.SIGKILL)
            except Exception:
                continue

def shutdown_handler(signum, frame):
    global gazebo_process
    print("\n[train_multi] Shutdown signal received. Cleaning up...")
    if gazebo_process:
        print("[train_multi] Terminating Gazebo and ROS 2 launch processes...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()
    sys.exit(0)

# Register shutdown handler
signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

# --- MAPPO Centralized Critic Definitions ---
OPPONENT_OBS = "opponent_obs"
OPPONENT_ACTION = "opponent_action"

class TorchCentralizedCriticModel(TorchModelV2, nn.Module):
    """Permutation-Invariant GNN / Mean-Embedding model with centralized critic."""

    def __init__(self, obs_space, action_space, num_outputs, model_config, name):
        TorchModelV2.__init__(
            self, obs_space, action_space, num_outputs, model_config, name
        )
        nn.Module.__init__(self)

        # 1. Neighbor embedding network
        # Relative dist and angle (2 dims) -> hidden neighbor features (16 dims)
        self.neighbor_mlp = nn.Sequential(
            SlimFC(2, 16, activation_fn=nn.ReLU),
            SlimFC(16, 16, activation_fn=nn.ReLU),
        )

        # 2. Main actor network: takes own state (28 dims) + aggregated neighbors (16 dims) = 44 dims
        self.actor_mlp = nn.Sequential(
            SlimFC(44, 64, activation_fn=nn.Tanh),
            SlimFC(64, 64, activation_fn=nn.Tanh),
            SlimFC(64, num_outputs),
        )

        # Local value function head (fallback/evaluation)
        self.local_vf = nn.Sequential(
            SlimFC(44, 64, activation_fn=nn.Tanh),
            SlimFC(64, 1),
        )

        # 3. Opponent embedding network (for centralized critic)
        # Process joint state of each opponent: obs (46) + action (2) = 48 dims
        self.opponent_mlp = nn.Sequential(
            SlimFC(48, 32, activation_fn=nn.ReLU),
            SlimFC(32, 32, activation_fn=nn.ReLU),
        )

        # 4. Centralized Critic: own features (44) + aggregated opponents (32) = 76 dims
        self.central_vf = nn.Sequential(
            SlimFC(76, 64, activation_fn=nn.Tanh),
            SlimFC(64, 64, activation_fn=nn.Tanh),
            SlimFC(64, 1),
        )

        self._features = None

    @override(TorchModelV2)
    def forward(self, input_dict, state, seq_lens):
        obs = input_dict["obs"]
        batch_size = obs.shape[0]

        # Extract ego features
        ego_feats = obs[:, 0:28]

        # Extract neighbor features
        neighbors = obs[:, 28:46].reshape(batch_size, 9, 2)
        neighbors_flat = neighbors.reshape(batch_size * 9, 2)
        neighbor_embeds_flat = self.neighbor_mlp(neighbors_flat)
        neighbor_embeds = neighbor_embeds_flat.reshape(batch_size, 9, 16)

        # Mask out padded neighbors (padded distance is 10.0)
        neighbor_dists = neighbors[:, :, 0:1]
        mask = (neighbor_dists < 9.0).float()
        neighbor_embeds = neighbor_embeds * mask
        num_valid = torch.clamp(mask.sum(dim=1), min=1.0)
        aggregated_neighbors = neighbor_embeds.sum(dim=1) / num_valid

        # Concatenate and pass to actor
        actor_input = torch.cat([ego_feats, aggregated_neighbors], dim=1)
        self._features = actor_input

        model_out = self.actor_mlp(actor_input)
        return model_out, []

    def central_value_function(self, obs, opponent_obs, opponent_actions):
        batch_size = obs.shape[0]

        # 1. Process main agent's own observation to own features (44 dims)
        ego_feats = obs[:, 0:28]
        neighbors = obs[:, 28:46].reshape(batch_size, 9, 2)
        neighbors_flat = neighbors.reshape(batch_size * 9, 2)
        neighbor_embeds_flat = self.neighbor_mlp(neighbors_flat)
        neighbor_embeds = neighbor_embeds_flat.reshape(batch_size, 9, 16)
        neighbor_dists = neighbors[:, :, 0:1]
        mask = (neighbor_dists < 9.0).float()
        neighbor_embeds = neighbor_embeds * mask
        num_valid = torch.clamp(mask.sum(dim=1), min=1.0)
        aggregated_neighbors = neighbor_embeds.sum(dim=1) / num_valid
        own_features = torch.cat([ego_feats, aggregated_neighbors], dim=1)

        # 2. Process opponents (max 9 opponents)
        opp_obs = opponent_obs.reshape(batch_size, 9, 46)
        opp_actions = opponent_actions.reshape(batch_size, 9, 2)
        opp_joint = torch.cat([opp_obs, opp_actions], dim=2)
        opp_joint_flat = opp_joint.reshape(batch_size * 9, 48)
        opp_embeds_flat = self.opponent_mlp(opp_joint_flat)
        opp_embeds = opp_embeds_flat.reshape(batch_size, 9, 32)

        # Mask out inactive/dead opponents (all zeros)
        opp_mask = (opp_obs.abs().sum(dim=2, keepdim=True) > 1e-3).float()
        opp_embeds = opp_embeds * opp_mask
        num_opp_valid = torch.clamp(opp_mask.sum(dim=1), min=1.0)
        aggregated_opponents = opp_embeds.sum(dim=1) / num_opp_valid

        # 3. Predict value
        central_input = torch.cat([own_features, aggregated_opponents], dim=1)
        return torch.reshape(self.central_vf(central_input), [-1])

    @override(TorchModelV2)
    def value_function(self):
        assert self._features is not None, "must call forward() first"
        return torch.reshape(self.local_vf(self._features), [-1])


class CentralizedValueMixin:
    """Add method to evaluate the central value function from the model."""
    def __init__(self):
        self.compute_central_vf = self.model.central_value_function


def centralized_critic_postprocessing(
    policy, sample_batch, other_agent_batches=None, episode=None
):
    pytorch = policy.config["framework"] == "torch"
    if pytorch and hasattr(policy, "compute_central_vf"):
        assert other_agent_batches is not None
        
        # Sort opponent keys to ensure deterministic order
        sorted_opponents = sorted(list(other_agent_batches.keys()))
        
        opp_obs_list = []
        opp_act_list = []
        batch_len = len(sample_batch[SampleBatch.CUR_OBS])
        
        for opp_id in sorted_opponents:
            opp_val = other_agent_batches[opp_id]
            opp_batch = opp_val[-1]
            opp_obs = opp_batch[SampleBatch.CUR_OBS]
            opp_act = opp_batch[SampleBatch.ACTIONS]
            
            # Pad or slice to match current batch length
            opp_len = len(opp_obs)
            if opp_len < batch_len:
                pad_obs = np.zeros((batch_len - opp_len, opp_obs.shape[1]), dtype=opp_obs.dtype)
                opp_obs = np.concatenate([opp_obs, pad_obs], axis=0)
                pad_act = np.zeros((batch_len - opp_len, opp_act.shape[1]), dtype=opp_act.dtype)
                opp_act = np.concatenate([opp_act, pad_act], axis=0)
            elif opp_len > batch_len:
                opp_obs = opp_obs[:batch_len]
                opp_act = opp_act[:batch_len]
                
            opp_obs_list.append(opp_obs)
            opp_act_list.append(opp_act)
            
        # Pad to exactly 9 opponents (total 10 robots)
        while len(opp_obs_list) < 9:
            opp_obs_list.append(np.zeros((batch_len, 46), dtype=np.float32))
            opp_act_list.append(np.zeros((batch_len, 2), dtype=np.float32))
            
        # Concatenate opponents' states
        opponent_obs = np.concatenate(opp_obs_list, axis=1) # Shape: (batch_size, 414)
        opponent_act = np.concatenate(opp_act_list, axis=1) # Shape: (batch_size, 18)
        
        sample_batch[OPPONENT_OBS] = opponent_obs
        sample_batch[OPPONENT_ACTION] = opponent_act
        
        # Evaluate centralized value predictions
        sample_batch[SampleBatch.VF_PREDS] = (
            policy.compute_central_vf(
                convert_to_torch_tensor(sample_batch[SampleBatch.CUR_OBS], policy.device),
                convert_to_torch_tensor(sample_batch[OPPONENT_OBS], policy.device),
                convert_to_torch_tensor(sample_batch[OPPONENT_ACTION], policy.device),
            )
            .cpu()
            .detach()
            .numpy()
        )
    else:
        # Policy initialization phase
        batch_len = len(sample_batch[SampleBatch.CUR_OBS])
        sample_batch[OPPONENT_OBS] = np.zeros((batch_len, 414), dtype=np.float32)
        sample_batch[OPPONENT_ACTION] = np.zeros((batch_len, 18), dtype=np.float32)
        sample_batch[SampleBatch.VF_PREDS] = np.zeros_like(
            sample_batch[SampleBatch.REWARDS], dtype=np.float32
        )

    completed = sample_batch[SampleBatch.TERMINATEDS][-1]
    if completed:
        last_r = 0.0
    else:
        last_r = sample_batch[SampleBatch.VF_PREDS][-1]

    train_batch = compute_advantages(
        sample_batch,
        last_r,
        policy.config["gamma"],
        policy.config["lambda"],
        use_gae=policy.config["use_gae"],
    )
    return train_batch


def loss_with_central_critic(policy, base_policy, model, dist_class, train_batch):
    # Save original value function.
    vf_saved = model.value_function

    # Temporarily bind model's value function to return the central value output
    model.value_function = lambda: policy.model.central_value_function(
        train_batch[SampleBatch.CUR_OBS],
        train_batch[OPPONENT_OBS],
        train_batch[OPPONENT_ACTION],
    )
    policy._central_value_out = model.value_function()
    loss = base_policy.loss(model, dist_class, train_batch)

    # Restore original value function.
    model.value_function = vf_saved
    return loss


class CCPPOTorchPolicy(CentralizedValueMixin, PPOTorchPolicy):
    def __init__(self, observation_space, action_space, config):
        PPOTorchPolicy.__init__(self, observation_space, action_space, config)
        CentralizedValueMixin.__init__(self)

    @override(PPOTorchPolicy)
    def loss(self, model, dist_class, train_batch):
        return loss_with_central_critic(self, super(), model, dist_class, train_batch)

    @override(PPOTorchPolicy)
    def postprocess_trajectory(
        self, sample_batch, other_agent_batches=None, episode=None
    ):
        return centralized_critic_postprocessing(
            self, sample_batch, other_agent_batches, episode
        )


class CentralizedCritic(PPO):
    @classmethod
    @override(PPO)
    def get_default_policy_class(cls, config):
        return CCPPOTorchPolicy


# --- MAPPO RLlib Training ---
def run_training(iterations=15, checkpoint_dir="./checkpoints", headless=True):
    import ray
    from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
    from ray.tune.registry import register_env
    
    # Register Custom Centralized Critic Model
    ModelCatalog.register_custom_model("cc_model", TorchCentralizedCriticModel)
    
    # 1. Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # 2. Register Environment
    def env_creator(config_dict):
        return ParallelPettingZooEnv(PettingZooSwarmEnv(max_steps=150))
        
    register_env("mars_swarm_v0", env_creator)
    
    # Extract observation and action spaces from temporary env
    print("[train_multi] Fetching environment dimensions...")
    temp_env = PettingZooSwarmEnv(max_steps=10)
    obs_space = temp_env.observation_space("tb1")
    act_space = temp_env.action_space("tb1")
    temp_env.close()
    
    # 3. Configure MAPPO
    # Since robots are homogeneous, we train a single shared policy
    config = (
        PPOConfig()
        .api_stack(
            enable_env_runner_and_connector_v2=False,
            enable_rl_module_and_learner=False,
        )
        .environment("mars_swarm_v0")
        .framework("torch")
        .env_runners(
            num_env_runners=0,  # Must be 0 to keep Gazebo running inside the main worker process
            rollout_fragment_length=150,
        )
        .training(
            model={"custom_model": "cc_model"},
            train_batch_size=300,
            minibatch_size=64,
            num_epochs=5,
            lr=1e-4,
            clip_param=0.2,
            gamma=0.99,
        )
        .multi_agent(
            policies={"shared_policy": (None, obs_space, act_space, {})},
            policy_mapping_fn=lambda agent_id, *args, **kwargs: "shared_policy",
        )
        .resources(num_gpus=0)
    )
    
    print("[train_multi] Starting MAPPO Training Loop with Centralized Critic...")
    start_gazebo(headless=headless)
    
    algo = CentralizedCritic(config=config)
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    for i in range(1, iterations + 1):
        result = algo.train()
        reward_mean = result.get('episode_reward_mean', float('nan'))
        
        # Policy loss details
        policy_stats = result.get('info', {}).get('learner', {}).get('shared_policy', {}).get('learner_stats', {})
        loss = policy_stats.get('policy_loss', 0.0)
        
        print(f"Iteration {i:2d}/{iterations} | Mean Swarm Reward: {reward_mean:.2f} | Policy Loss: {loss:.4f}")
        
        # Save checkpoints periodically
        if i % 5 == 0:
            checkpoint_path = algo.save(checkpoint_dir)
            print(f"[train_multi] Saved checkpoint: {checkpoint_path}")
            
    algo.stop()
    print("[train_multi] Training finished successfully!")
    
    if gazebo_process:
        print("[train_multi] Stopping Gazebo...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()

# --- Evaluation Node ---
def run_evaluation(checkpoint_path, episodes=5, headless=False):
    import ray
    from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
    from ray.tune.registry import register_env
    
    # Register Custom Centralized Critic Model
    ModelCatalog.register_custom_model("cc_model", TorchCentralizedCriticModel)
    
    # 1. Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # 2. Register Environment
    def env_creator(config_dict):
        return ParallelPettingZooEnv(PettingZooSwarmEnv(max_steps=150))
        
    register_env("mars_swarm_v0", env_creator)
    
    # 3. Load Algorithm from Checkpoint
    print(f"[train_multi] Loading Algorithm from checkpoint: {checkpoint_path}")
    algo = CentralizedCritic.from_checkpoint(checkpoint_path)
    
    # 4. Start Gazebo
    start_gazebo(headless=headless)
    
    print("[train_multi] Initializing PettingZoo Swarm Environment for Evaluation...")
    env = PettingZooSwarmEnv(max_steps=150)
    
    print(f"[train_multi] --- Running Multi-Agent Swarm Evaluation (Episodes: {episodes}) ---")
    
    for ep in range(1, episodes + 1):
        obs, infos = env.reset()
        ep_rewards = {agent: 0.0 for agent in env.possible_agents}
        steps = 0
        
        print(f"\nEvaluation Episode {ep} Started.")
        for agent in env.possible_agents:
            print(f"  {agent} Target Goal: {env.goal_positions[agent]}")
            
        active = True
        while active and steps < 150:
            actions = {}
            for agent in env.agents:
                policy = algo.get_policy("shared_policy")
                agent_obs = obs[agent]
                act_batch, _, _ = policy.compute_actions(
                    np.array([agent_obs]),
                    explore=False
                )
                actions[agent] = act_batch[0]
                
            obs, rewards, terminations, truncations, infos = env.step(actions)
            
            for agent in rewards:
                ep_rewards[agent] += rewards[agent]
                
            steps += 1
            if len(env.agents) == 0:
                active = False
                
            if steps % 20 == 0:
                status_str = " | ".join([
                    f"{agent}: ({infos[agent]['x']:.2f}, {infos[agent]['y']:.2f}) d={infos[agent]['goal_dist']:.2f}"
                    for agent in env.possible_agents if agent in infos
                ])
                print(f"  Step {steps:3d} | {status_str}")
                
        print(f"Evaluation Episode {ep} Finished | Steps: {steps}")
        for agent in env.possible_agents:
            status = infos[agent]['status'] if agent in infos else 'INACTIVE'
            print(f"  {agent} Reward: {ep_rewards[agent]:6.2f} | Status: {status}")
            
    env.close()
    if gazebo_process:
        print("[train_multi] Stopping Gazebo...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()

# --- Main Entry Point ---
def main():
    parser = argparse.ArgumentParser(description="Multi-Agent Swarm Navigation for TurtleBot3 Swarm")
    parser.add_argument('--demo', action='store_true', help="Run random action demo mode")
    parser.add_argument('--train', action='store_true', help="Run Ray RLlib MAPPO training")
    parser.add_argument('--evaluate', action='store_true', help="Evaluate a trained MAPPO policy checkpoint")
    parser.add_argument('--checkpoint', type=str, default="", help="Path to checkpoint directory for evaluation")
    parser.add_argument('--episodes', type=int, default=5, help="Number of evaluation episodes")
    parser.add_argument('--iterations', type=int, default=15, help="Number of training iterations")
    parser.add_argument('--gui', action='store_true', help="Run Gazebo with GUI enabled (not headless)")
    args = parser.parse_args()
    
    if args.evaluate:
        if not args.checkpoint:
            print("[ERROR] Please specify a checkpoint path using --checkpoint <path>")
            sys.exit(1)
        run_evaluation(checkpoint_path=args.checkpoint, episodes=args.episodes, headless=not args.gui)
    elif args.train:
        run_training(iterations=args.iterations, headless=not args.gui)
    else:
        # Run demo mode
        start_gazebo(headless=not args.gui)
        
        print("[train_multi] Initializing PettingZoo Swarm Environment...")
        env = PettingZooSwarmEnv(max_steps=150)
        
        print("[train_multi] --- Running Multi-Agent Swarm Demo Mode ---")
        
        for ep in range(1, 6):
            obs, infos = env.reset()
            ep_rewards = {agent: 0.0 for agent in env.possible_agents}
            steps = 0
            
            print(f"\nEpisode {ep} Started.")
            for agent in env.possible_agents:
                print(f"  {agent} Target Goal: {env.goal_positions[agent]}")
                
            active = True
            while active and steps < 150:
                actions = {}
                for agent in env.agents:
                    actions[agent] = env.action_space(agent).sample()
                    
                obs, rewards, terminations, truncations, infos = env.step(actions)
                
                for agent in rewards:
                    ep_rewards[agent] += rewards[agent]
                    
                steps += 1
                
                if len(env.agents) == 0:
                    active = False
                    
                if steps % 20 == 0:
                    status_str = " | ".join([
                        f"{agent}: ({infos[agent]['x']:.2f}, {infos[agent]['y']:.2f}) d={infos[agent]['goal_dist']:.2f}"
                        for agent in env.possible_agents if agent in infos
                    ])
                    print(f"  Step {steps:3d} | {status_str}")
                    
            print(f"Episode {ep} Finished | Steps: {steps}")
            for agent in env.possible_agents:
                status = infos[agent]['status'] if agent in infos else 'INACTIVE'
                print(f"  {agent} Reward: {ep_rewards[agent]:6.2f} | Status: {status}")
                
        env.close()
        if gazebo_process:
            print("[train_multi] Stopping Gazebo...")
            try:
                os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
                gazebo_process.wait(timeout=3)
            except Exception:
                pass
        kill_stale_processes()

if __name__ == "__main__":
    main()
