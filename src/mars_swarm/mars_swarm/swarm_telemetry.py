"""
Comprehensive Swarm Telemetry & Run Logging.

Exports structured telemetry data (run_summary.json and TensorBoard events) capturing:
1. Area Coverage Rate curve (ACR vs. Step)
2. Normalized Energy Consumption (integral of v^2 + omega^2 dt per meter traveled)
3. Mean Time Between Deadlocks (MTBD)
4. Minimum Inter-Robot Clearance Histogram
"""

import os
import json
import math
import numpy as np

try:
    from torch.utils.tensorboard import SummaryWriter
    HAS_TENSORBOARD = True
except ImportError:
    HAS_TENSORBOARD = False


class SwarmTelemetryLogger:
    """
    Real-time telemetry logger and metrics aggregator for multi-agent swarm runs.
    """
    
    def __init__(self, log_dir="checkpoints/telemetry", enable_tensorboard=True, dt=0.1):
        self.log_dir = log_dir
        self.enable_tensorboard = enable_tensorboard and HAS_TENSORBOARD
        self.dt = float(dt)
        
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.writer = None
        if self.enable_tensorboard:
            try:
                self.writer = SummaryWriter(log_dir=self.log_dir)
            except Exception as e:
                print(f"[telemetry] [WARNING] Failed to initialize TensorBoard SummaryWriter: {e}")
                self.writer = None
                
        self.reset()
        
    def reset(self):
        """Reset internal telemetry state for a new episode."""
        self.steps = 0
        self.acr_curve = []  # [(step, acr)]
        self.cumulative_energy = 0.0
        self.deadlock_timestamps = []
        self.clearances_history = []
        self.collision_events = 0
        self.wall_collision_events = 0
        self.agent_collision_events = 0
        
    def record_step(self, step, acr, actions_dict, poses_dict, is_deadlock_event=False,
                    collisions=0, wall_collisions=0, agent_collisions=0):
        """
        Record telemetry snapshot for a single simulation step.
        """
        self.steps = step
        self.acr_curve.append((int(step), float(round(acr, 2))))
        self.collision_events = collisions
        self.wall_collision_events = wall_collisions
        self.agent_collision_events = agent_collisions
        
        # 1. Energy consumption: sum(v^2 + omega^2) * dt across agents
        step_energy = 0.0
        for agent, act in actions_dict.items():
            if act is not None and len(act) >= 2:
                v = float(act[0])
                w = float(act[1])
                step_energy += (v**2 + w**2) * self.dt
        self.cumulative_energy += step_energy
        
        # 2. Minimum Inter-Robot Clearance
        active_agents = [a for a in poses_dict if poses_dict[a] is not None]
        if len(active_agents) >= 2:
            min_dist = float('inf')
            for i in range(len(active_agents)):
                a1 = active_agents[i]
                x1, y1 = poses_dict[a1][0], poses_dict[a1][1]
                for j in range(i + 1, len(active_agents)):
                    a2 = active_agents[j]
                    x2, y2 = poses_dict[a2][0], poses_dict[a2][1]
                    d = math.hypot(x1 - x2, y1 - y2)
                    if d < min_dist:
                        min_dist = d
            if min_dist < float('inf'):
                self.clearances_history.append(float(min_dist))
                
        # 3. Deadlock Tracking
        if is_deadlock_event:
            self.deadlock_timestamps.append(int(step))
            
        # 4. Optional TensorBoard streaming every 10 steps
        if self.writer and step % 10 == 0:
            self.writer.add_scalar("Swarm/ACR", acr, step)
            self.writer.add_scalar("Swarm/CumulativeEnergy", self.cumulative_energy, step)
            if len(self.clearances_history) > 0:
                self.writer.add_scalar("Swarm/MinClearance", self.clearances_history[-1], step)
            self.writer.add_scalar("Swarm/WallCollisions", self.wall_collision_events, step)
            self.writer.add_scalar("Swarm/AgentCollisions", self.agent_collision_events, step)
                
    def finalize_and_export(self, final_results, export_path=None):
        """
        Calculates aggregate summary metrics, exports run_summary.json,
        and flushes TensorBoard summaries.
        """
        total_distance = float(final_results.get('distance', 0.0))
        redundancy = float(final_results.get('redundancy', 0.0))
        final_acr = float(final_results.get('acr', 0.0))
        
        # Normalized energy (energy per meter traveled)
        normalized_energy = (self.cumulative_energy / max(1e-3, total_distance))
        
        # Mean Time Between Deadlocks (MTBD in steps)
        num_deadlocks = len(self.deadlock_timestamps)
        if num_deadlocks == 0:
            mtbd_steps = float(self.steps)
        elif num_deadlocks == 1:
            mtbd_steps = float(self.steps)
        else:
            intervals = np.diff(self.deadlock_timestamps)
            mtbd_steps = float(np.mean(intervals))
            
        # Clearance Histogram (10 bins from 0.0 to 3.0 meters)
        clearance_arr = np.array(self.clearances_history, dtype=np.float32) if len(self.clearances_history) > 0 else np.array([0.0])
        hist_counts, bin_edges = np.histogram(clearance_arr, bins=10, range=(0.0, 3.0))
        
        total_coll = int(final_results.get('collisions', self.collision_events))
        wall_coll = int(final_results.get('wall_collisions', self.wall_collision_events))
        agent_coll = int(final_results.get('agent_collisions', self.agent_collision_events))

        summary_payload = {
            "final_acr_percent": round(final_acr, 2),
            "total_steps": int(self.steps),
            "total_distance_meters": round(total_distance, 2),
            "cumulative_energy_joules_proxy": round(self.cumulative_energy, 3),
            "normalized_energy_j_per_m": round(normalized_energy, 4),
            "cell_overlap_redundancy": round(redundancy, 2),
            "total_collisions": total_coll,
            "wall_collisions": wall_coll,
            "agent_collisions": agent_coll,
            "deadlock_count": int(num_deadlocks),
            "mean_time_between_deadlocks_steps": round(mtbd_steps, 1),
            "inter_robot_clearance": {
                "min_meters": round(float(np.min(clearance_arr)), 3),
                "mean_meters": round(float(np.mean(clearance_arr)), 3),
                "histogram_bins": [round(float(b), 2) for b in bin_edges],
                "histogram_counts": [int(c) for c in hist_counts]
            },
            "acr_curve_sampled": self.acr_curve[::max(1, len(self.acr_curve) // 100)]  # max 100 sampled points
        }
        
        target_json = export_path or os.path.join(self.log_dir, "run_summary.json")
        with open(target_json, 'w') as f:
            json.dump(summary_payload, f, indent=2)
            
        print(f"\n[telemetry] Structured run telemetry exported to: {target_json}")
        
        # Log final scalars to TensorBoard
        if self.writer:
            self.writer.add_scalar("Final/ACR", final_acr, self.steps)
            self.writer.add_scalar("Final/NormalizedEnergy", normalized_energy, self.steps)
            self.writer.add_scalar("Final/MTBD", mtbd_steps, self.steps)
            self.writer.add_scalar("Final/Collisions", self.collision_events, self.steps)
            if len(clearance_arr) > 0:
                self.writer.add_histogram("Final/ClearanceDistribution", clearance_arr, self.steps)
            self.writer.flush()
            self.writer.close()
            
        return summary_payload
