"""Unit and regression tests for Control Barrier Function (CBF) & ACAS collision avoidance."""

import pytest
import numpy as np
from scipy.optimize import minimize


def apply_cbf_standalone(obs, v_nom, w_nom):
    """Standalone reference implementation of multi_env_wrapper._apply_cbf."""
    if obs is None:
        return v_nom, w_nom
        
    lidar_ranges = obs[0:24]
    neighbors = obs[28:46].reshape(9, 2)
    
    l = 0.12
    d_safe = 0.20
    gamma = 2.0
    
    def objective(u):
        return (u[0] - v_nom)**2 + 0.05 * (u[1] - w_nom)**2
        
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
            'fun': lambda u, A=A_front, B=B_front, h_val=h_front: A * u[0] + B * u[1] + gamma * h_val
        })
        
        # Rear lookahead
        h_rear = (-l - d_j * np.cos(phi_j))**2 + (d_j * np.sin(phi_j))**2 - d_safe**2
        A_rear = 2 * (-l - d_j * np.cos(phi_j))
        B_rear = 2 * l * d_j * np.sin(phi_j)
        
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_rear, B=B_rear, h_val=h_rear: A * u[0] + B * u[1] + gamma * h_val
        })
        
    for dist, angle in neighbors:
        if dist > 0.6 or dist < 0.01:
            continue
        h_front = (l - dist * np.cos(angle))**2 + (dist * np.sin(angle))**2 - d_safe**2
        A_front = 2 * (l - dist * np.cos(angle))
        B_front = -2 * l * dist * np.sin(angle)
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_front, B=B_front, h_val=h_front: A * u[0] + B * u[1] + gamma * h_val
        })
        
        h_rear = (-l - dist * np.cos(angle))**2 + (dist * np.sin(angle))**2 - d_safe**2
        A_rear = 2 * (-l - dist * np.cos(angle))
        B_rear = 2 * l * dist * np.sin(angle)
        cons.append({
            'type': 'ineq',
            'fun': lambda u, A=A_rear, B=B_rear, h_val=h_rear: A * u[0] + B * u[1] + gamma * h_val
        })
        
    bounds = [(-0.22, 0.22), (-1.0, 1.0)]
    res = minimize(objective, x0=np.array([v_nom, w_nom]), method='SLSQP', bounds=bounds, constraints=cons)
    
    if res.success:
        return float(res.x[0]), float(res.x[1])
    else:
        # Fixed fallback: halt linear velocity to prevent obstacle-charging
        return 0.0, w_nom


def test_cbf_free_space():
    """In free space (no obstacles within 0.6m), CBF should preserve nominal velocity."""
    obs = np.ones(46, dtype=np.float32) * 5.0
    obs[28:46] = 10.0  # Far neighbors
    
    v_safe, w_safe = apply_cbf_standalone(obs, v_nom=0.20, w_nom=0.10)
    assert np.isclose(v_safe, 0.20, atol=1e-2)
    assert np.isclose(w_safe, 0.10, atol=1e-2)


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
    # Forward velocity must be 0.0 or negative (never positive into wall)
    assert v_safe <= 0.0
    assert np.isclose(w_safe, 0.50, atol=1e-3)
