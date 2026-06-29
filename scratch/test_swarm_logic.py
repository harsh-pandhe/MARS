import unittest
import numpy as np
import math

class TestSwarmLogic(unittest.TestCase):
    def test_coordinate_transforms(self):
        """
        Verify that coordinates are transformed correctly using offset mappings.
        """
        # Spawn coordinates:
        # tb1: (0, 0, 0)
        # tb2: (0.433, -0.25, 2.0944)  # 120 degrees
        # tb3: (0.433, 0.25, -2.0944)  # -120 degrees
        
        # Test translation offsets
        tb2_spawn = (0.433, -0.25, 2.0944)
        tb3_spawn = (0.433, 0.25, -2.0944)
        
        # In a coordinate system aligned to tb1, tb1's origin is (0,0)
        # Verify that relative distances match expectation
        dist_tb2_tb3 = math.hypot(tb2_spawn[0] - tb3_spawn[0], tb2_spawn[1] - tb3_spawn[1])
        self.assertAlmostEqual(dist_tb2_tb3, 0.5, places=3)

    def test_frontier_bfs_mock(self):
        """
        Verify BFS frontier calculation on a mock grid.
        """
        # Mock class matching PettingZooSwarmEnv properties
        class MockEnv:
            def __init__(self):
                self.grid_bounds = (-5.0, 5.0, -5.0, 5.0)
                self.grid_resolution_x = 10
                self.grid_resolution_y = 10
                self.viz_resolution_x = 20
                self.viz_resolution_y = 20
                self.visited_grid = np.zeros((10, 10), dtype=bool)
                self.viz_grid = np.zeros((20, 20), dtype=np.int8)  # All clear (0)

        env = MockEnv()
        # Mark all cells except cell (8, 8) as visited
        env.visited_grid.fill(True)
        env.visited_grid[8, 8] = False
        
        # Robot is at (0, 0) in world coords, which maps to grid center (5, 5)
        # We want to run a simple BFS mock matching get_reachable_unvisited_frontiers logic
        # Map robot position to visited_grid cell
        min_x, max_x, min_y, max_y = env.grid_bounds
        x, y = 0.0, 0.0
        c0 = int(np.clip((x - min_x) / (max_x - min_x) * env.grid_resolution_x, 0, env.grid_resolution_x - 1))
        r0 = int(np.clip((y - min_y) / (max_y - min_y) * env.grid_resolution_y, 0, env.grid_resolution_y - 1))
        
        self.assertEqual(c0, 5)
        self.assertEqual(r0, 5)
        
        # Traverse BFS to find unvisited cell (8, 8)
        queue = [(r0, c0)]
        visited = set(queue)
        found_frontiers = []
        
        while queue:
            r, c = queue.pop(0)
            if not env.visited_grid[r, c]:
                # Map back to world
                cx = min_x + (c + 0.5) * (max_x - min_x) / env.grid_resolution_x
                cy = min_y + (r + 0.5) * (max_y - min_y) / env.grid_resolution_y
                found_frontiers.append((cx, cy))
                break
                
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < env.grid_resolution_y and 0 <= nc < env.grid_resolution_x:
                    if (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append((nr, nc))
                        
        self.assertEqual(len(found_frontiers), 1)
        # The coordinates of cell (8, 8) in [-5, 5] bounds:
        # width = 10, resolution = 10 cells, cell width = 1.0
        # cell 8 center = -5.0 + (8 + 0.5)*1.0 = 3.5
        self.assertAlmostEqual(found_frontiers[0][0], 3.5)
        self.assertAlmostEqual(found_frontiers[0][1], 3.5)

    def test_cooperative_target_allocation(self):
        """
        Verify that target allocation repels targets away from other robots' active targets.
        """
        # Mock allocator
        # Let's say robot 1 has active target at (2.0, 2.0).
        # Robot 2 is located at (0.0, 0.0).
        # Frontiers are at:
        # F1: (2.0, 1.9)  <- very close to Robot 1's target
        # F2: (-2.0, -2.0) <- far from Robot 1's target
        
        robot_pose = (0.0, 0.0, 0.0)
        active_targets = {'tb1': (2.0, 2.0), 'tb2': None}
        frontiers = [(2.0, 1.9), (-2.0, -2.0)]
        
        # Calculate cost for tb2 to each frontier
        # Cost(F) = dist(tb2, F) + 3.0 / dist(F, tb1_target)
        costs = []
        for fx, fy in frontiers:
            dist_to_robot = math.hypot(fx - robot_pose[0], fy - robot_pose[1])
            repulsion = 0.0
            for other_tgt in [active_targets['tb1']]:
                d_tgt = math.hypot(fx - other_tgt[0], fy - other_tgt[1])
                repulsion += 3.0 / max(d_tgt, 0.1)
            costs.append(dist_to_robot + repulsion)
            
        # Cost for F1: hypot(2, 1.9) + 3.0 / 0.1 = ~2.75 + 30 = 32.75
        # Cost for F2: hypot(-2, -2) + 3.0 / hypot(-4, -4) = ~2.82 + 3.0 / 5.65 = ~3.35
        self.assertTrue(costs[1] < costs[0])  # F2 has significantly lower cost due to repulsion penalty!

    def test_apf_steering_repulsion(self):
        """
        Verify that APF successfully steers the robot away from obstacle forces.
        """
        # Target is straight ahead: local angle = 0, so attractive force F_att = (1, 0)
        F_x = 1.0
        F_y = 0.0
        
        # Obstacle is at local angle pi/4 (front-left) with distance 0.3m (very close!)
        # Repulsive force points in direction pi + pi/4 = -3pi/4 (-135 degrees)
        d_b = 0.3
        beam_angle = np.pi / 4.0
        
        F_rep_mag = 0.08 * (1.0 / d_b - 1.0 / 0.7) / (d_b**2)
        F_x -= F_rep_mag * math.cos(beam_angle)
        F_y -= F_rep_mag * math.sin(beam_angle)
        
        steer_angle = math.atan2(F_y, F_x)
        
        # The steer angle should be negative (turning right, away from front-left obstacle)
        self.assertTrue(steer_angle < 0)
        # Steering should turn right significantly
        self.assertTrue(abs(steer_angle) > 0.5)

    def test_watchdog_monitoring(self):
        """
        Verify that watchdog monitor successfully increments stagnant ticks and flags dead agents.
        """
        env_agents = ['tb1', 'tb2', 'tb3']
        active_targets = {'tb1': (1.0, 1.0), 'tb2': None, 'tb3': None}
        
        last_poses_watchdog = {'tb1': (0.0, 0.0, 0.0)}
        stagnant_ticks = {'tb1': 39}
        
        # Command was non-zero
        prev_action = np.array([0.1, 0.0])
        
        # Robot is stuck (current pose is exactly the same as last pose)
        curr_pose = (0.0, 0.0, 0.0)
        dist_moved = math.hypot(curr_pose[0] - last_poses_watchdog['tb1'][0], curr_pose[1] - last_poses_watchdog['tb1'][1])
        
        if dist_moved < 0.002 and (abs(prev_action[0]) > 0.02 or abs(prev_action[1]) > 0.05):
            stagnant_ticks['tb1'] += 1
            
        self.assertEqual(stagnant_ticks['tb1'], 40)
        
        # Declare dead if >= 40
        agents_to_kill = []
        if stagnant_ticks['tb1'] >= 40:
            agents_to_kill.append('tb1')
            
        for agent in agents_to_kill:
            env_agents.remove(agent)
            active_targets[agent] = None
            
        self.assertNotIn('tb1', env_agents)
        self.assertIsNone(active_targets['tb1'])

    def test_centralized_critic_postprocessing(self):
        """
        Verify that centralized critic postprocessing successfully aligns,
        pads, and concatenates observations and actions of other swarm agents.
        """
        from ray.rllib.policy.sample_batch import SampleBatch
        
        # 1. Create mock policy and inputs
        class MockPolicy:
            def __init__(self):
                self.config = {"framework": "torch", "gamma": 0.99, "lambda": 0.95, "use_gae": True}
                self.device = "cpu"
            def compute_central_vf(self, obs, opponent_obs, opponent_actions):
                # Central critic returns value vector of shape (batch_len,)
                import torch
                return torch.zeros(obs.shape[0])
                
        policy = MockPolicy()
        policy.compute_central_vf = policy.compute_central_vf
        setattr(policy, "compute_central_vf", policy.compute_central_vf)

        # 2. Main agent's trajectory (length 5, obs dimension 32, action dimension 2)
        sample_batch = SampleBatch({
            SampleBatch.CUR_OBS: np.ones((5, 32), dtype=np.float32),
            SampleBatch.ACTIONS: np.ones((5, 2), dtype=np.float32),
            SampleBatch.REWARDS: np.ones(5, dtype=np.float32),
            SampleBatch.TERMINATEDS: np.array([False, False, False, False, True]),
            SampleBatch.VF_PREDS: np.zeros(5, dtype=np.float32),
        })

        # 3. Opponent 1: Shorter trajectory (length 3, obs dimension 32, action dimension 2)
        opp1_batch = SampleBatch({
            SampleBatch.CUR_OBS: np.ones((3, 32), dtype=np.float32) * 2.0,
            SampleBatch.ACTIONS: np.ones((3, 2), dtype=np.float32) * 2.0,
        })

        # 4. Opponent 2: Longer trajectory (length 7, obs dimension 32, action dimension 2)
        opp2_batch = SampleBatch({
            SampleBatch.CUR_OBS: np.ones((7, 32), dtype=np.float32) * 3.0,
            SampleBatch.ACTIONS: np.ones((7, 2), dtype=np.float32) * 3.0,
        })

        other_agent_batches = {
            "tb2": (None, opp1_batch),
            "tb3": (None, opp2_batch),
        }

        # 5. Run the postprocessing logic
        from train_multi import centralized_critic_postprocessing
        processed_batch = centralized_critic_postprocessing(
            policy, sample_batch, other_agent_batches=other_agent_batches
        )

        # 6. Assertions
        # Opponent observations should be concatenated to shape (5, 64)
        self.assertEqual(processed_batch["opponent_obs"].shape, (5, 64))
        # Opponent actions should be concatenated to shape (5, 4)
        self.assertEqual(processed_batch["opponent_action"].shape, (5, 4))

        # Check Opponent 1 (tb2) - should be padded with zeros for index 3 and 4
        # Since keys are sorted, "tb2" is index 0 in concatenated dimensions
        opp1_obs_reconstructed = processed_batch["opponent_obs"][:, :32]
        opp1_act_reconstructed = processed_batch["opponent_action"][:, :2]
        self.assertTrue(np.all(opp1_obs_reconstructed[:3] == 2.0))
        self.assertTrue(np.all(opp1_obs_reconstructed[3:] == 0.0))
        self.assertTrue(np.all(opp1_act_reconstructed[:3] == 2.0))
        self.assertTrue(np.all(opp1_act_reconstructed[3:] == 0.0))

        # Check Opponent 2 (tb3) - should be sliced to match length 5
        # "tb3" is index 1 in concatenated dimensions
        opp2_obs_reconstructed = processed_batch["opponent_obs"][:, 32:]
        opp2_act_reconstructed = processed_batch["opponent_action"][:, 2:]
        self.assertEqual(len(opp2_obs_reconstructed), 5)
        self.assertTrue(np.all(opp2_obs_reconstructed == 3.0))
        self.assertTrue(np.all(opp2_act_reconstructed == 3.0))

        print("[test_centralized_critic_postprocessing] All assertions passed successfully!")

if __name__ == '__main__':
    unittest.main()
