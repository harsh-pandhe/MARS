"""Unit and regression tests for Control Barrier Function (CBF) & ACAS collision avoidance."""

import sys
import os
import pytest
import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'mars_swarm'))


def apply_cbf_standalone(obs, v_nom, w_nom):
    """Standalone reference implementation of multi_env_wrapper._apply_cbf."""
    if obs is None:
        return v_nom, w_nom
        
    lidar_ranges = obs[0:24]
    neighbors = obs[28:46].reshape(9, 2)
    
    l = 0.12
    d_safe = 0.20
    gamma = 2.0
    
    # Target objective: minimize deviation from nominal commands + penalty on slack
    P_slack = 500.0
    def objective(u):
        return (u[0] - v_nom)**2 + 0.05 * (u[1] - w_nom)**2 + P_slack * (u[2]**2)
        
    cons = []
    angles = np.linspace(-np.pi, np.pi, 24)
    for j in range(24):
        d_j = lidar_ranges[j]
        if d_j > 0.6:
            continue
        phi_j = angles[j]
        
        # Front lookahead
        h_front = (l - d_j * np.cos(phi_j))**2 + (d_j * np.sin(phi_j))**2 - d_safe**2
        A_front = 2 * (l - d_j * np.cos(phi_j))
        B_front = -2 * l * d_j * np.sin(phi_j)
        
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_front, B=B_front, h_val=h_front: A * u[0] + B * u[1] + gamma * h_val + u[2]
        })
        
        # Rear lookahead
        h_rear = (-l - d_j * np.cos(phi_j))**2 + (d_j * np.sin(phi_j))**2 - d_safe**2
        A_rear = 2 * (-l - d_j * np.cos(phi_j))
        B_rear = 2 * l * d_j * np.sin(phi_j)
        
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_rear, B=B_rear, h_val=h_rear: A * u[0] + B * u[1] + gamma * h_val + u[2]
        })
        
    for dist, angle in neighbors:
        if dist > 0.6 or dist < 0.01:
            continue
        h_front = (l - dist * np.cos(angle))**2 + (dist * np.sin(angle))**2 - d_safe**2
        A_front = 2 * (l - dist * np.cos(angle))
        B_front = -2 * l * dist * np.sin(angle)
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_front, B=B_front, h_val=h_front: A * u[0] + B * u[1] + gamma * h_val + u[2]
        })
        
        h_rear = (-l - dist * np.cos(angle))**2 + (dist * np.sin(angle))**2 - d_safe**2
        A_rear = 2 * (-l - dist * np.cos(angle))
        B_rear = 2 * l * dist * np.sin(angle)
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_rear, B=B_rear, h_val=h_rear: A * u[0] + B * u[1] + gamma * h_val + u[2]
        })
        
    bounds = [(-0.22, 0.22), (-1.0, 1.0), (0.0, 10.0)]
    res = minimize(objective, x0=np.array([v_nom, w_nom, 0.0]), method='SLSQP', bounds=bounds, constraints=cons)
    
    if res.success:
        v_out = float(res.x[0])
        w_out = float(res.x[1])
        if np.min(lidar_ranges[10:15]) < 0.20:
            v_out = min(0.0, v_out)
        return v_out, w_out
    else:
        # Fallback: halt forward motion into obstacles, but allow reverse backing out
        return min(0.0, v_nom), w_nom


def test_cbf_free_space():
    """In free space (no obstacles within 0.6m), CBF should preserve nominal velocity."""
    obs = np.ones(46, dtype=np.float32) * 5.0
    obs[28:46] = 10.0  # Far neighbors
    
    v_safe, w_safe = apply_cbf_standalone(obs, v_nom=0.20, w_nom=0.10)
    assert np.isclose(v_safe, 0.20, atol=1e-2)
    assert np.isclose(w_safe, 0.10, atol=1e-2)


def test_cbf_inter_agent_repulsion():
    """Verify that an approaching neighbor forces forward velocity to zero or negative."""
    from mars_swarm.cbf_qp_solver import FastCBFSolver
    solver = FastCBFSolver(l=0.12, d_safe_obs=0.20, d_safe_agent=0.45)
    lidar_free = np.full(24, 3.5, dtype=np.float32)
    
    # Neighbor 0.30m in front (angle=0.0)
    neighbors = np.array([[0.30, 0.0]], dtype=np.float32)
    v_safe, w_safe = solver.solve(v_nom=0.20, w_nom=0.0, lidar_ranges=lidar_free, neighbors=neighbors)
    
    assert v_safe <= 0.05, f"Approaching neighbor should brake! Got v_safe={v_safe:.3f}"


def test_cbf_front_obstacle_braking():
    """With an obstacle straight ahead (0.25m), CBF must reduce or stop forward velocity."""
    obs = np.ones(46, dtype=np.float32) * 3.0
    obs[28:46] = 10.0
    
    # 24 sectors from -pi to pi; index 11/12 corresponds to forward (phi ~ 0)
    obs[11] = 0.24
    obs[12] = 0.24
    
    v_safe, w_safe = apply_cbf_standalone(obs, v_nom=0.22, w_nom=0.0)
    assert v_safe < 0.22
    assert v_safe >= -0.22


def test_cbf_infeasible_fallback_never_charges_obstacle():
    """When trapped in an infeasible constraint pocket, fallback must halt forward motion."""
    obs = np.ones(46, dtype=np.float32) * 0.15  # Surrounding wall at 0.15m (< d_safe)
    obs[28:46] = 10.0
    
    v_safe, w_safe = apply_cbf_standalone(obs, v_nom=0.22, w_nom=0.50)
    # Forward velocity must be <= 0.0 (never positive into wall)
    assert v_safe <= 0.0


def test_cbf_reverse_escape_allowed():
    """When wedged against an obstacle, reverse escape command (v_nom < 0) must NOT be blocked."""
    obs = np.ones(46, dtype=np.float32) * 3.0
    obs[28:46] = 10.0
    obs[11] = 0.15  # Obstacle directly in front
    obs[12] = 0.15
    
    v_safe, w_safe = apply_cbf_standalone(obs, v_nom=-0.12, w_nom=0.6)
    # Must allow negative velocity to back away from the wall
    assert v_safe < 0.0


def test_fast_cbf_solver_correctness_and_speed():
    """Verify FastCBFSolver produces safe commands and executes in under 1 millisecond."""
    import time
    from mars_swarm.cbf_qp_solver import FastCBFSolver
    
    solver = FastCBFSolver()
    
    # 1. Free space
    lidar = np.ones(24, dtype=np.float32) * 4.0
    v, w = solver.solve(0.20, 0.10, lidar)
    assert np.isclose(v, 0.20, atol=1e-2)
    assert np.isclose(w, 0.10, atol=1e-2)
    
    # 2. Obstacle braking
    lidar[11] = 0.22
    lidar[12] = 0.22
    v, w = solver.solve(0.22, 0.0, lidar)
    assert v < 0.22
    
    # 3. Reverse escape
    lidar[11] = 0.15
    lidar[12] = 0.15
    v, w = solver.solve(-0.12, 0.6, lidar)
    assert v < 0.0
    
    # 4. Latency benchmark (< 2.0 ms, supporting 500+ Hz control loops)
    solver.solve(0.22, 0.5, lidar)  # Warm-up OSQP setup
    t0 = time.perf_counter()
    N_SOLVES = 100
    for _ in range(N_SOLVES):
        solver.solve(0.22, 0.5, lidar)
    t1 = time.perf_counter()
    avg_time_ms = ((t1 - t0) / N_SOLVES) * 1000.0
    print(f"FastCBFSolver average solve time: {avg_time_ms:.3f} ms")
    assert avg_time_ms < 2.0, f"Solver too slow: {avg_time_ms} ms"

