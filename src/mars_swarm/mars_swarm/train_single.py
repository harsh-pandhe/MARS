import os
import sys
import time
import math
import argparse
import subprocess
import signal
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

# Add package directory to path to allow direct execution
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mars_swarm.env_wrapper import TurtleBot3Env

# Set device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# --- Actor-Critic Neural Network ---
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()
        
        # Shared or separate backbones. We use separate backbones for actor and critic for stability.
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, action_dim),
            nn.Tanh()  # Output in [-1, 1]
        )
        
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )
        
        # Action log standard deviation (parameter for exploration)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        raise NotImplementedError

    def get_action_and_value(self, state, action=None):
        action_mean = self.actor(state)
        action_std = torch.exp(self.log_std)
        dist = Normal(action_mean, action_std)
        
        if action is None:
            action = dist.sample()
            
        log_prob = dist.log_prob(action).sum(axis=-1)
        entropy = dist.entropy().sum(axis=-1)
        value = self.critic(state)
        
        return action, log_prob, entropy, value

# --- Memory Buffer ---
class RolloutBuffer:
    def __init__(self):
        self.states = []
        self.actions = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
        self.values = []

    def clear(self):
        del self.states[:]
        del self.actions[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]
        del self.values[:]

# --- PPO Agent ---
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor=3e-4, lr_critic=1e-3, gamma=0.99, K_epochs=40, eps_clip=0.2, c1=0.5, c2=0.01):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        
        self.c1 = c1  # Value loss weight
        self.c2 = c2  # Entropy coefficient
        
        self.policy = ActorCritic(state_dim, action_dim).to(device)
        self.optimizer = optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic},
            {'params': [self.policy.log_std], 'lr': lr_actor}
        ])
        
        self.policy_old = ActorCritic(state_dim, action_dim).to(device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()

    def select_action(self, state, buffer):
        with torch.no_grad():
            state_t = torch.FloatTensor(state).to(device)
            action, logprob, _, value = self.policy_old.get_action_and_value(state_t)
            
        buffer.states.append(state)
        buffer.actions.append(action.cpu().numpy())
        buffer.logprobs.append(logprob.item())
        buffer.values.append(value.item())
        
        return action.cpu().numpy()

    def update(self, buffer):
        # Monte Carlo estimate of returns
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(buffer.rewards), reversed(buffer.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        # Normalize the rewards
        rewards = torch.FloatTensor(rewards).to(device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)
        
        # Convert list to tensor
        old_states = torch.FloatTensor(np.array(buffer.states)).to(device)
        old_actions = torch.FloatTensor(np.array(buffer.actions)).to(device)
        old_logprobs = torch.FloatTensor(np.array(buffer.logprobs)).to(device)
        old_values = torch.FloatTensor(np.array(buffer.values)).to(device)
        
        # Optimize policy for K epochs
        for _ in range(self.K_epochs):
            # Evaluating old actions and values
            _, logprobs, entropy, state_values = self.policy.get_action_and_value(old_states, old_actions)
            
            # Match state_values tensor dimensions with rewards tensor
            state_values = torch.squeeze(state_values)
            
            # Finding the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(logprobs - old_logprobs)
            
            # Finding Surrogate Loss
            advantages = rewards - old_values.detach()
            # Normalize advantages
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-7)
            
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1.0 - self.eps_clip, 1.0 + self.eps_clip) * advantages
            
            # Final loss of PPO
            loss = -torch.min(surr1, surr2) + self.c1 * self.MseLoss(state_values, rewards) - self.c2 * entropy
            
            # Take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
        # Copy new weights to old policy
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        # Clear buffer
        buffer.clear()

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)
        print(f"Model saved to {checkpoint_path}")

    def load(self, checkpoint_path):
        self.policy_old.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))
        self.policy.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))
        print(f"Model loaded from {checkpoint_path}")

# --- Gazebo Launcher Helpers ---
gazebo_process = None

def start_gazebo(headless=True):
    global gazebo_process
    print("[train_single] Starting Gazebo simulation in background...")
    # Source ROS 2 setup
    cmd = [
        "ros2", "launch", "mars_swarm", "spawn_single.launch.py",
        f"headless:={str(headless).lower()}"
    ]
    # Launch in a separate process group so we can terminate the entire tree
    gazebo_process = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid
    )
    print("[train_single] Gazebo process launched. Waiting for topics...")
    time.sleep(8.0)  # Wait for Gazebo and ROS 2 bridges to fully start up

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
                target_terms = ['gz sim', 'parameter_bridge', 'ros_gz_bridge', 'spawn_single.launch.py', 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz']
                if any(term in lower_cmd for term in target_terms):
                    os.kill(pid, signal.SIGKILL)
            except Exception:
                continue

def shutdown_handler(signum, frame):
    global gazebo_process
    print("\n[train_single] Shutdown signal received. Cleaning up...")
    if gazebo_process:
        print("[train_single] Terminating Gazebo and ROS 2 launch processes...")
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

# --- Main Entry Point ---
def main():
    parser = argparse.ArgumentParser(description="PPO Point-to-Point Navigation for TurtleBot3 Waffle")
    parser.add_argument("--demo", action="store_true", help="Run in evaluation demo mode with current/pre-trained policy")
    parser.add_argument("--headless", type=str, default="true", help="Run Gazebo headless ('true' or 'false')")
    parser.add_argument("--episodes", type=int, default=100, help="Number of training episodes")
    parser.add_argument("--checkpoint", type=str, default="ppo_single_tb3.pt", help="Path to save/load checkpoint")
    args = parser.parse_args()
    
    headless_bool = args.headless.lower() == "true"
    
    # 1. Start Gazebo Simulation
    start_gazebo(headless=headless_bool)
    
    # 2. Instantiate Environment
    print("[train_single] Initializing TurtleBot3 Gymnasium environment...")
    env = TurtleBot3Env(max_steps=250)
    
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    # 3. Initialize Agent
    agent = PPOAgent(state_dim, action_dim)
    buffer = RolloutBuffer()
    
    checkpoint_dir = "checkpoints"
    os.makedirs(checkpoint_dir, exist_ok=True)
    checkpoint_path = os.path.join(checkpoint_dir, args.checkpoint)
    
    if os.path.exists(checkpoint_path):
        agent.load(checkpoint_path)
    
    if args.demo:
        print("[train_single] --- Running Demo Mode ---")
        for ep in range(5):
            state, info = env.reset()
            ep_reward = 0
            done = False
            step = 0
            
            print(f"\nEpisode {ep + 1} Started. Target Goal: {env.goal_pos}")
            while not done:
                action = agent.select_action(state, buffer)
                # Clear buffer immediately as we don't update in demo mode
                buffer.clear()
                
                state, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                ep_reward += reward
                step += 1
                
                if step % 20 == 0:
                    print(f"  Step {step:3d} | Robot Pos: ({info['x']:.2f}, {info['y']:.2f}) | Goal Dist: {info['goal_dist']:.2f}")
                    
                time.sleep(0.05)  # Add sleep to run demo close to real-time speed
                
            status = "SUCCESS" if not info.get('collision', False) and info['goal_dist'] < 0.35 else "FAILED"
            print(f"Episode {ep + 1} Finished | Steps: {step} | Reward: {ep_reward:.2f} | Status: {status}")
            
    else:
        print("[train_single] --- Running Training Mode ---")
        update_timestep = 1000  # Update policy every 1000 steps
        timestep = 0
        
        for ep in range(1, args.episodes + 1):
            state, info = env.reset()
            ep_reward = 0
            done = False
            step = 0
            
            while not done:
                action = agent.select_action(state, buffer)
                state, reward, terminated, truncated, info = env.step(action)
                
                buffer.rewards.append(reward)
                buffer.is_terminals.append(terminated)
                
                done = terminated or truncated
                ep_reward += reward
                timestep += 1
                step += 1
                
                # Perform policy update
                if timestep % update_timestep == 0:
                    print(f"\n[Step {timestep}] Performing PPO update...")
                    agent.update(buffer)
                    
            print(f"Episode {ep:3d} | Steps: {step:3d} | Total Reward: {ep_reward:6.2f} | Goal Dist: {info['goal_dist']:.2f}")
            
            # Save checkpoint every 10 episodes
            if ep % 10 == 0:
                agent.save(checkpoint_path)
                
    # Cleanup
    env.close()
    if gazebo_process:
        print("[train_single] Stopping Gazebo...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()

if __name__ == "__main__":
    main()
