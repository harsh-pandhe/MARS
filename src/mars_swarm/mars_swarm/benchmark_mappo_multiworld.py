#!/usr/bin/env python3
"""
MARS Swarm Multi-World MAPPO & Baseline Comparison Benchmark Suite.

Executes standardized 150-step benchmarking evaluations across all 5 worlds:
  - cafe
  - warehouse
  - depot
  - office
  - maze

Under 5 baseline conditions:
  1. Frontier Heuristic (Dynamic Voronoi + A* + BT + CBF)
  2. Random Walk
  3. MAPPO (Nominal)
  4. MAPPO (Sensor Noise - Gaussian N(0, 0.15) on 24-beam LiDAR)
  5. MAPPO (Agent Failure - tb2 disabled at step 50)

Evaluates cross-world generalization and quantifies penalty-avoidance policy freezing.
"""

import os
os.environ["TORCHDYNAMO_DISABLE"] = "1"
os.environ["TORCH_COMPILE_DISABLE"] = "1"
os.environ["ENABLE_RAY_CLUSTER"] = "0"
os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0"

import sys
import time
import signal
import json
import argparse
import numpy as np

# Ensure workspace install directory is in AMENT_PREFIX_PATH
workspace_install = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../install/mars_swarm"))
if os.path.exists(workspace_install):
    cur_ament = os.environ.get("AMENT_PREFIX_PATH", "")
    if workspace_install not in cur_ament:
        os.environ["AMENT_PREFIX_PATH"] = f"{workspace_install}:{cur_ament}" if cur_ament else workspace_install

# Ensure workspace packages are accessible
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from multi_env_wrapper import PettingZooSwarmEnv
from evaluate_benchmarks import run_benchmark_episode
from train_multi import start_gazebo, kill_stale_processes, gazebo_process

SCENARIOS = {
    'heuristic': {
        'name': 'Frontier Heuristic',
        'mode': 'heuristic',
        'noise': False,
        'failure': False,
    },
    'random': {
        'name': 'Random Walk',
        'mode': 'random',
        'noise': False,
        'failure': False,
    },
    'mappo_nominal': {
        'name': 'MAPPO (Nominal)',
        'mode': 'mappo',
        'noise': False,
        'failure': False,
    },
    'mappo_noise': {
        'name': 'MAPPO (Sensor Noise)',
        'mode': 'mappo',
        'noise': True,
        'failure': False,
    },
    'mappo_failure': {
        'name': 'MAPPO (Agent Failure)',
        'mode': 'mappo',
        'noise': False,
        'failure': True,
    },
}

DEFAULT_WORLDS = ['cafe', 'warehouse', 'depot', 'office', 'maze']


def run_single_benchmark(world='cafe', scenario_key='heuristic', max_steps=150,
                         num_robots=3, seed=42, checkpoint="", headless=True):
    if scenario_key not in SCENARIOS:
        raise ValueError(f"Unknown scenario key: {scenario_key}")

    scen = SCENARIOS[scenario_key]
    print("\n" + "=" * 70)
    print(f"  BENCHMARK RUN: WORLD={world.upper()} | SCENARIO={scen['name']} | STEPS={max_steps} | SEED={seed}")
    print("=" * 70 + "\n")

    policy = None
    if scen['mode'] == 'mappo':
        if not checkpoint:
            checkpoint = "checkpoints/mappo_baseline/policies/shared_policy"
        ckpt_path = os.path.abspath(checkpoint)
        if not os.path.isdir(ckpt_path) or not os.path.exists(os.path.join(ckpt_path, "policy_state.pkl")):
            # Try subfolder
            cand = os.path.join(ckpt_path, "policies", "shared_policy")
            if os.path.isdir(cand):
                ckpt_path = cand

        print(f"[benchmark] Loading MAPPO checkpoint from: {ckpt_path}")
        from ray.rllib.models import ModelCatalog
        from train_multi import TorchCentralizedCriticModel
        ModelCatalog.register_custom_model("cc_model", TorchCentralizedCriticModel)
        from ray.rllib.policy.policy import Policy
        policy = Policy.from_checkpoint(ckpt_path)
        print("[benchmark] MAPPO policy loaded successfully.")

    start_gazebo(headless=headless, world=world, seed=seed, num_robots=num_robots)

    print(f"[benchmark] Initializing Swarm Environment for {num_robots} robots in '{world}'...")
    env = PettingZooSwarmEnv(
        num_robots=num_robots,
        max_steps=max_steps,
        continuous_exploration=True,
        world=world
    )

    t0 = time.time()
    res = run_benchmark_episode(
        env,
        policy=policy,
        mode=scen['mode'],
        inject_noise=scen['noise'],
        inject_failure=scen['failure'],
        verbose=True
    )
    elapsed = time.time() - t0

    env.close()
    if gazebo_process:
        print("[benchmark] Terminating Gazebo simulation...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()
    time.sleep(2.0)

    summary = {
        'scenario_key': scenario_key,
        'scenario_name': scen['name'],
        'world': world,
        'num_robots': num_robots,
        'max_steps': max_steps,
        'seed': seed,
        'steps': res['steps'],
        'elapsed_sec': round(elapsed, 2),
        'acr': round(res['acr'], 2),
        'total_distance_m': round(res['distance'], 2),
        'dist_per_robot_m': round(res['distance'] / max(1, num_robots), 2),
        'redundancy': round(res['redundancy'], 2),
        'total_collisions': res['collisions'],
        'wall_collisions': res['wall_collisions'],
        'agent_collisions': res['agent_collisions'],
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="MARS Swarm Multi-World MAPPO & Baseline Benchmark")
    parser.add_argument('--worlds', type=str, nargs='+', default=DEFAULT_WORLDS,
                        choices=['cafe', 'warehouse', 'depot', 'office', 'maze', 'all'],
                        help="List of benchmark worlds to evaluate")
    parser.add_argument('--scenarios', type=str, nargs='+', default=list(SCENARIOS.keys()),
                        choices=list(SCENARIOS.keys()) + ['all', 'mappo_only'],
                        help="List of scenarios to evaluate")
    parser.add_argument('--max-steps', type=int, default=150,
                        help="Evaluation horizon per episode (default: 150)")
    parser.add_argument('--num-robots', type=int, default=3,
                        help="Swarm size (default: 3)")
    parser.add_argument('--seed', type=int, default=42,
                        help="PRNG seed for deterministic evaluation")
    parser.add_argument('--checkpoint', type=str, default="checkpoints/mappo_baseline/policies/shared_policy",
                        help="Path to MAPPO policy checkpoint")
    parser.add_argument('--gui', action='store_true',
                        help="Run Gazebo with GUI enabled")
    parser.add_argument('--output-json', type=str, default="checkpoints/mappo_multiworld_comparison.json",
                        help="Path to export consolidated multi-world benchmark results")
    parser.add_argument('--force', action='store_true',
                        help="Force re-evaluation of previously cached world/scenario pairs")

    # Internal flags for isolated child subprocess
    parser.add_argument('--single-run', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--single-world', type=str, default="", help=argparse.SUPPRESS)
    parser.add_argument('--single-scenario', type=str, default="", help=argparse.SUPPRESS)
    parser.add_argument('--export-single', type=str, default="", help=argparse.SUPPRESS)
    args = parser.parse_args()

    # If executing isolated single run
    if args.single_run:
        world = args.single_world or 'cafe'
        scenario = args.single_scenario or 'heuristic'
        summary = run_single_benchmark(
            world=world,
            scenario_key=scenario,
            max_steps=args.max_steps,
            num_robots=args.num_robots,
            seed=args.seed,
            checkpoint=args.checkpoint,
            headless=not args.gui
        )
        if args.export_single:
            with open(args.export_single, 'w') as f:
                json.dump(summary, f)
        return

    import subprocess
    import tempfile

    # Parse worlds
    if 'all' in args.worlds:
        worlds = DEFAULT_WORLDS
    else:
        worlds = args.worlds

    # Parse scenarios
    if 'all' in args.scenarios:
        scenarios = list(SCENARIOS.keys())
    elif 'mappo_only' in args.scenarios:
        scenarios = ['mappo_nominal', 'mappo_noise', 'mappo_failure']
    else:
        scenarios = args.scenarios

    # Load existing results if any
    data = {
        'metadata': {
            'max_steps': args.max_steps,
            'num_robots': args.num_robots,
            'seed': args.seed,
            'checkpoint': args.checkpoint,
        },
        'results': {w: {} for w in DEFAULT_WORLDS}
    }

    if not args.force and args.output_json and os.path.exists(args.output_json):
        try:
            with open(args.output_json, 'r') as f:
                prev = json.load(f)
                if 'results' in prev and isinstance(prev['results'], dict):
                    for w, scens in prev['results'].items():
                        if w in data['results']:
                            data['results'][w].update(scens)
                    print(f"[orchestrator] Loaded existing benchmark results from: {args.output_json}")
        except Exception as e:
            print(f"[orchestrator] Warning reading existing JSON: {e}")

    total_runs = len(worlds) * len(scenarios)
    run_idx = 0

    print("\n" + "#" * 70)
    print("  MARS MULTI-WORLD MAPPO & BASELINE COMPARISON SUITE")
    print(f"  Worlds ({len(worlds)}):     {', '.join(worlds)}")
    print(f"  Scenarios ({len(scenarios)}):  {', '.join(scenarios)}")
    print(f"  Horizon:        {args.max_steps} steps | Swarm: {args.num_robots} robots | Seed: {args.seed}")
    print(f"  Output JSON:    {args.output_json}")
    print("#" * 70)

    for w in worlds:
        print(f"\n{'='*70}\n  WORLD: {w.upper()}\n{'='*70}")
        for s in scenarios:
            run_idx += 1
            scen_name = SCENARIOS[s]['name']

            # Check if already completed
            if not args.force and w in data['results'] and s in data['results'][w]:
                prev_entry = data['results'][w][s]
                print(f"\n>>> [{run_idx}/{total_runs}] {scen_name} in '{w}' already completed "
                      f"({prev_entry.get('acr')} % ACR, {prev_entry.get('total_distance_m')} m). Skipping.")
                continue

            print(f"\n>>> [{run_idx}/{total_runs}] Launching isolated subprocess for '{scen_name}' in '{w}'...")

            with tempfile.NamedTemporaryFile(suffix=f'_mappo_{w}_{s}.json', delete=False) as tmp:
                tmp_path = tmp.name

            cmd = [
                sys.executable, os.path.abspath(__file__),
                '--single-run',
                '--single-world', w,
                '--single-scenario', s,
                '--max-steps', str(args.max_steps),
                '--num-robots', str(args.num_robots),
                '--seed', str(args.seed),
                '--checkpoint', args.checkpoint,
                '--export-single', tmp_path
            ]
            if args.gui:
                cmd.append('--gui')

            sub_env = os.environ.copy()
            if os.path.exists(workspace_install):
                cur_a = sub_env.get("AMENT_PREFIX_PATH", "")
                if workspace_install not in cur_a:
                    sub_env["AMENT_PREFIX_PATH"] = f"{workspace_install}:{cur_a}" if cur_a else workspace_install

            ret = subprocess.run(cmd, env=sub_env)
            if ret.returncode == 0 and os.path.exists(tmp_path):
                try:
                    with open(tmp_path, 'r') as f:
                        summary = json.load(f)
                    if w not in data['results']:
                        data['results'][w] = {}
                    data['results'][w][s] = summary
                    print(f"[orchestrator] SUCCESS: {scen_name} in '{w}' -> ACR: {summary['acr']}%, "
                          f"Dist: {summary['total_distance_m']}m, Collisions: {summary['total_collisions']}")
                except Exception as e:
                    print(f"[orchestrator] ERROR reading single run output: {e}")
                finally:
                    if os.path.exists(tmp_path):
                        os.unlink(tmp_path)
            else:
                print(f"[orchestrator] FAILED: returncode={ret.returncode}")

            # Save progress incrementally
            if args.output_json:
                os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
                with open(args.output_json, 'w') as f:
                    json.dump(data, f, indent=2)

    # Print Consolidated Summary Table
    print("\n\n" + "=" * 90)
    print("                 MULTI-WORLD MAPPO BASELINE COMPARISON SUMMARY MATRIX")
    print("=" * 90)
    header = f"{'World':12s} | {'Scenario':24s} | {'ACR (%)':8s} | {'Redundancy':10s} | {'Dist (m)':8s} | {'Dist/Rob':8s} | {'Collisions':10s}"
    print(header)
    print("-" * 90)

    for w in DEFAULT_WORLDS:
        if w not in data['results'] or not data['results'][w]:
            continue
        for s in SCENARIOS.keys():
            if s in data['results'][w]:
                entry = data['results'][w][s]
                line = (f"{w:12s} | {entry['scenario_name']:24s} | {entry['acr']:7.1f}% | "
                        f"{entry['redundancy']:10.2f} | {entry['total_distance_m']:7.2f}m | "
                        f"{entry['dist_per_robot_m']:7.2f}m | {entry['total_collisions']:4d} (W:{entry['wall_collisions']}, A:{entry['agent_collisions']})")
                print(line)
        print("-" * 90)

    print(f"\nConsolidated comparison matrix successfully exported to: {args.output_json}\n")


if __name__ == "__main__":
    main()
