#!/usr/bin/env python3
"""
MARS Swarm Robot Count Scalability Sweep Harness.

Runs automated exploration benchmarks across varying robot counts (e.g. N=2, 3, 5, 8)
in any benchmark world (cafe, warehouse, depot, office, maze) using the decentralized
frontier heuristic, evaluating:
  - Area Coverage Rate (ACR %)
  - Cumulative & Per-Robot Distance Traveled (Energy/Transit)
  - Coverage Redundancy
  - Inter-agent vs. Wall Collision Rates
"""

import os
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


def run_single_sweep(world='depot', num_robots=3, max_steps=300, seed=42, headless=True):
    print("\n" + "=" * 60)
    print(f"  SWEEP EVALUATION: {num_robots} ROBOTS | WORLD: {world.upper()} | STEPS: {max_steps} | SEED: {seed}")
    print("=" * 60 + "\n")

    start_gazebo(headless=headless, world=world, seed=seed, num_robots=num_robots)

    print(f"[sweep] Initializing Swarm Environment for {num_robots} robots...")
    env = PettingZooSwarmEnv(
        num_robots=num_robots,
        max_steps=max_steps,
        continuous_exploration=True,
        world=world
    )

    t0 = time.time()
    res = run_benchmark_episode(env, mode='heuristic', verbose=True)
    elapsed = time.time() - t0

    env.close()
    if gazebo_process:
        print("[sweep] Terminating Gazebo simulation...")
        try:
            os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
            gazebo_process.wait(timeout=3)
        except Exception:
            pass
    kill_stale_processes()
    time.sleep(2.0)

    summary = {
        'num_robots': num_robots,
        'world': world,
        'max_steps': max_steps,
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
    parser = argparse.ArgumentParser(description="MARS Swarm Robot Count Scalability Sweep")
    parser.add_argument('--world', type=str, default='depot',
                        choices=['cafe', 'warehouse', 'depot', 'office', 'maze', 'all'],
                        help="Benchmark world environment or 'all' for all 5 worlds")
    parser.add_argument('--worlds', type=str, nargs='+', default=None,
                        help="List of benchmark worlds to sweep (e.g. --worlds cafe depot maze)")
    parser.add_argument('--robot-counts', type=int, nargs='+', default=[2, 3, 5, 8],
                        help="List of swarm sizes to evaluate (1 to 8)")
    parser.add_argument('--max-steps', type=int, default=300,
                        help="Number of steps per evaluation episode")
    parser.add_argument('--seed', type=int, default=42,
                        help="PRNG seed for repeatability")
    parser.add_argument('--gui', action='store_true',
                        help="Run Gazebo with GUI enabled")
    parser.add_argument('--output-json', type=str, default="checkpoints/sweep_scaling_results.json",
                        help="Filepath to export sweep scaling data")
    parser.add_argument('--force', action='store_true',
                        help="Force re-running all configurations, ignoring cached results")
    parser.add_argument('--single-run', action='store_true',
                        help=argparse.SUPPRESS)
    parser.add_argument('--num-robots', type=int, default=3,
                        help=argparse.SUPPRESS)
    parser.add_argument('--export-single', type=str, default="",
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    # If executing single isolated subprocess run
    if args.single_run:
        summary = run_single_sweep(
            world=args.world,
            num_robots=args.num_robots,
            max_steps=args.max_steps,
            seed=args.seed,
            headless=not args.gui
        )
        if args.export_single:
            with open(args.export_single, 'w') as f:
                json.dump(summary, f)
        return

    import subprocess
    import tempfile

    if args.worlds:
        worlds_to_sweep = args.worlds
    elif args.world == 'all':
        worlds_to_sweep = ['cafe', 'warehouse', 'depot', 'office', 'maze']
    else:
        worlds_to_sweep = [args.world]

    all_results = {}
    if not args.force and args.output_json and os.path.exists(args.output_json):
        try:
            with open(args.output_json, 'r') as f:
                prev = json.load(f)
                if 'all_results' in prev and isinstance(prev['all_results'], dict):
                    all_results = prev['all_results']
                    print(f"[sweep] Loaded existing results from: {args.output_json}")
        except Exception as e:
            print(f"[sweep] Warning reading existing JSON: {e}")

    total_runs = len(worlds_to_sweep) * len([n for n in args.robot_counts if 1 <= n <= 8])
    run_idx = 0

    print("\n" + "#" * 70)
    print(f"  STARTING ROBOT-COUNT SWEEP ACROSS {len(worlds_to_sweep)} WORLD(S)")
    print(f"  Worlds:       {', '.join(worlds_to_sweep)}")
    print(f"  Robot Counts: {args.robot_counts}")
    print(f"  Max Steps:    {args.max_steps} | Seed: {args.seed}")
    print("#" * 70)

    for current_world in worlds_to_sweep:
        if current_world not in all_results:
            all_results[current_world] = []
        print(f"\n{'='*70}\n  BEGIN SWEEP FOR WORLD: {current_world.upper()}\n{'='*70}")

        for n in args.robot_counts:
            if n < 1 or n > 8:
                print(f"[sweep] WARNING: Robot count {n} out of supported range [1, 8]. Skipping.")
                continue

            run_idx += 1

            # Check if this configuration was already completed
            existing = [r for r in all_results[current_world] if r.get('num_robots') == n and r.get('max_steps') == args.max_steps]
            if existing and not args.force:
                print(f"\n>>> [{run_idx}/{total_runs}] N={n} in '{current_world}' already completed ({existing[0].get('acr')} % ACR). Skipping.")
                continue

            print(f"\n>>> [{run_idx}/{total_runs}] Launching isolated runner for N={n} in '{current_world}'...")

            with tempfile.NamedTemporaryFile(suffix=f'_sweep_{current_world}_{n}.json', delete=False) as tmp:
                tmp_path = tmp.name

            cmd = [
                sys.executable, os.path.abspath(__file__),
                '--single-run',
                '--num-robots', str(n),
                '--world', current_world,
                '--max-steps', str(args.max_steps),
                '--seed', str(args.seed),
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
                with open(tmp_path, 'r') as f:
                    summary = json.load(f)
                all_results[current_world].append(summary)
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

                # Incremental export
                if args.output_json:
                    os.makedirs(os.path.dirname(args.output_json) or '.', exist_ok=True)
                    export_data = {
                        'worlds': worlds_to_sweep,
                        'max_steps': args.max_steps,
                        'seed': args.seed,
                        'all_results': all_results
                    }
                    if len(worlds_to_sweep) == 1:
                        export_data['world'] = worlds_to_sweep[0]
                        export_data['results'] = all_results[worlds_to_sweep[0]]
                    with open(args.output_json, 'w') as f:
                        json.dump(export_data, f, indent=2)
            else:
                print(f"[sweep] ERROR: Subprocess for N={n} in '{current_world}' failed (exit={ret.returncode})")

        # Print summary table for this world immediately
        res = all_results.get(current_world, [])
        if res:
            print("\n" + "=" * 80)
            print(f"               SWEEP SCALABILITY REPORT: {current_world.upper()} ({args.max_steps} steps)")
            print("=" * 80)
            header = f"{'Robots':>6} | {'ACR (%)':>8} | {'Tot Dist (m)':>12} | {'Dist/Rob (m)':>12} | {'Redundancy':>10} | {'Collisions (W / A)':>18}"
            print(header)
            print("-" * 80)
            for r in res:
                col_str = f"{r['wall_collisions']} / {r['agent_collisions']}"
                print(f"{r['num_robots']:>6} | {r['acr']:>7.1f}% | {r['total_distance_m']:>12.2f} | {r['dist_per_robot_m']:>12.2f} | {r['redundancy']:>10.2f} | {col_str:>18}")
            print("=" * 80)

    # Consolidated Grand Summary Table across all worlds
    if len(worlds_to_sweep) > 1:
        print("\n" + "#" * 90)
        print(f"                      GRAND MULTI-WORLD SCALABILITY SUMMARY ({args.max_steps} steps)")
        print("#" * 90)
        header = f"{'World':>12} | {'Robots':>6} | {'ACR (%)':>8} | {'Tot Dist':>10} | {'Dist/Rob':>10} | {'Redundancy':>10} | {'Collisions (W/A)':>16}"
        print(header)
        print("-" * 90)
        for w in worlds_to_sweep:
            for r in all_results.get(w, []):
                col_str = f"{r['wall_collisions']}/{r['agent_collisions']}"
                print(f"{w:>12} | {r['num_robots']:>6} | {r['acr']:>7.1f}% | {r['total_distance_m']:>10.1f} | {r['dist_per_robot_m']:>10.1f} | {r['redundancy']:>10.2f} | {col_str:>16}")
            print("-" * 90)
        print("#" * 90 + "\n")

    print(f"\n[sweep] Scalability data exported to: {args.output_json}")


if __name__ == '__main__':
    main()
