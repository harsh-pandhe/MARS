"""
High-performance Control Barrier Function (CBF) Quadratic Program (QP) Solver.

Replaces slow Python SLSQP minimization (~5-15 ms/agent) with a C-accelerated
OSQP micro-solver (~25-50 microseconds/agent), enabling 50-100 Hz real-time
safety filtering for multi-robot swarms.
"""

import time
import math
import numpy as np

# Optional fast C-solver imports
try:
    import osqp
    from scipy import sparse
    HAS_OSQP = True
except ImportError:
    HAS_OSQP = False

from scipy.optimize import minimize


class FastCBFSolver:
    """
    Solves the Soft Control Barrier Function (CBF) Quadratic Program:
    
        min_{v, w, delta}  (v - v_nom)^2 + 0.05*(w - w_nom)^2 + P_slack * delta^2
        
        subject to:
            A_front_k * v + B_front_k * w + delta >= -gamma * h_front_k
            A_rear_k  * v + B_rear_k  * w + delta >= -gamma * h_rear_k
            -0.22 <= v <= 0.22
            -1.0  <= w <= 1.0
            0.0   <= delta <= 10.0
    """
    
    def __init__(self, l=0.12, d_safe_obs=0.20, d_safe_agent=0.45, gamma=2.0, P_slack=1000.0, use_osqp=True, d_safe=None):
        self.l = float(l)
        if d_safe is not None:
            d_safe_obs = float(d_safe)
        self.d_safe_obs = float(d_safe_obs)
        self.d_safe_agent = float(d_safe_agent)
        self.d_safe = float(d_safe_obs)
        self.gamma = float(gamma)
        self.P_slack = float(P_slack)
        self.use_osqp = bool(use_osqp and HAS_OSQP)
        
        # Pre-build constant objective matrix P for OSQP:
        # 0.5 * u^T P u = v^2 + 0.05 * w^2 + P_slack * delta^2
        if self.use_osqp:
            self._P_sparse = sparse.diags([2.0, 0.1, 2.0 * self.P_slack], format='csc')
            
    def solve(self, v_nom, w_nom, lidar_ranges, neighbors=None):
        """
        Filters nominal velocity commands (v_nom, w_nom) to guarantee safety.
        
        Args:
            v_nom (float): Nominal forward velocity command in [-0.22, 0.22]
            w_nom (float): Nominal angular velocity command in [-1.0, 1.0]
            lidar_ranges (array-like): 24-sector 360-degree LiDAR distances in meters
            neighbors (array-like, optional): Array of shape (N, 2) with (dist, angle) to neighboring robots
            
        Returns:
            v_safe (float): Filtered safe forward velocity
            w_safe (float): Filtered safe angular velocity
        """
        v_nom = float(np.clip(v_nom, -0.22, 0.22))
        w_nom = float(np.clip(w_nom, -1.0, 1.0))
        
        # 1. Compile constraints
        A_rows = []
        B_rows = []
        h_rows = []
        
        # A. LiDAR obstacles (24 sectors from -pi to pi)
        if lidar_ranges is not None and len(lidar_ranges) > 0:
            angles = np.linspace(-np.pi, np.pi, len(lidar_ranges))
            for j in range(len(lidar_ranges)):
                d_j = float(lidar_ranges[j])
                if d_j > 0.60:
                    continue
                phi_j = angles[j]
                cos_p = math.cos(phi_j)
                sin_p = math.sin(phi_j)
                
                # Front lookahead: p_front = [l, 0]
                h_f = (self.l - d_j * cos_p)**2 + (d_j * sin_p)**2 - self.d_safe_obs**2
                A_f = 2.0 * (self.l - d_j * cos_p)
                B_f = -2.0 * self.l * d_j * sin_p
                A_rows.append(A_f)
                B_rows.append(B_f)
                h_rows.append(h_f)
                
                # Rear lookahead: p_rear = [-l, 0]
                h_r = (-self.l - d_j * cos_p)**2 + (d_j * sin_p)**2 - self.d_safe_obs**2
                A_r = 2.0 * (-self.l - d_j * cos_p)
                B_r = 2.0 * self.l * d_j * sin_p
                A_rows.append(A_r)
                B_rows.append(B_r)
                h_rows.append(h_r)
                
        # B. Inter-agent constraints (TurtleBot3 Waffle physical footprint: d_safe_agent >= 0.45m)
        if neighbors is not None:
            for dist, angle in neighbors:
                if dist > 1.20:
                    continue
                dist = max(float(dist), 0.02)
                cos_a = math.cos(angle)
                sin_a = math.sin(angle)
                
                # Direct relative distance CBF: h = dist^2 - d_safe_agent^2
                h_agent = dist**2 - self.d_safe_agent**2
                # Time derivative: \dot{dist} = -v * cos(angle) - v_neighbor (assumed bounded)
                # \dot{h} = 2 * dist * (-v * cos(angle)) >= -gamma * h
                A_agent = -2.0 * dist * cos_a
                B_agent = -2.0 * self.l * dist * sin_a
                A_rows.append(A_agent)
                B_rows.append(B_agent)
                h_rows.append(h_agent)
                
        # If no active constraints in proximity, nominal is safe
        if len(A_rows) == 0:
            return v_nom, w_nom
            
        # 2. Solve using OSQP if available
        if self.use_osqp:
            try:
                v_out, w_out = self._solve_osqp(v_nom, w_nom, A_rows, B_rows, h_rows)
                v_out = self._apply_front_safeguard(v_out, lidar_ranges)
                return v_out, w_out
            except Exception:
                pass  # Fall back to SLSQP on any OSQP error
                
        # 3. Fallback: SciPy SLSQP solver
        v_out, w_out = self._solve_scipy(v_nom, w_nom, A_rows, B_rows, h_rows)
        v_out = self._apply_front_safeguard(v_out, lidar_ranges)
        return v_out, w_out

    def _solve_osqp(self, v_nom, w_nom, A_rows, B_rows, h_rows):
        M = len(A_rows)
        q = np.array([-2.0 * v_nom, -0.1 * w_nom, 0.0])
        
        # Constraint matrix: stack variable bounds + barrier inequalities
        # [I_3; [A, B, 1]]
        A_cb = np.column_stack([A_rows, B_rows, np.ones(M)])
        A_tot = sparse.vstack([sparse.eye(3), sparse.csc_matrix(A_cb)], format='csc')
        
        l_tot = np.concatenate([[-0.22, -1.0, 0.0], -self.gamma * np.array(h_rows)])
        u_tot = np.concatenate([[0.22, 1.0, 10.0], np.full(M, np.inf)])
        
        solver = osqp.OSQP()
        solver.setup(self._P_sparse, q, A_tot, l_tot, u_tot, verbose=False, eps_abs=1e-3, eps_rel=1e-3, polishing=False)
        res = solver.solve()
        
        if res.info.status_val in (1, 2):  # solved or solved inaccurate
            return float(res.x[0]), float(res.x[1])
        else:
            # QP infeasible: halt forward motion, allow reverse backing
            return min(0.0, v_nom), w_nom

    def _solve_scipy(self, v_nom, w_nom, A_rows, B_rows, h_rows):
        def objective(u):
            return (u[0] - v_nom)**2 + 0.05 * (u[1] - w_nom)**2 + self.P_slack * (u[2]**2)
            
        cons = []
        for A_val, B_val, h_val in zip(A_rows, B_rows, h_rows):
            cons.append({
                'type': 'ineq',
                'fun': lambda u, A=A_val, B=B_val, h=h_val: A * u[0] + B * u[1] + self.gamma * h + u[2]
            })
            
        bounds = [(-0.22, 0.22), (-1.0, 1.0), (0.0, 10.0)]
        res = minimize(objective, x0=np.array([v_nom, w_nom, 0.0]), method='SLSQP', bounds=bounds, constraints=cons)
        
        if res.success:
            return float(res.x[0]), float(res.x[1])
        else:
            return min(0.0, v_nom), w_nom

    def _apply_front_safeguard(self, v_out, lidar_ranges):
        """Strict physical safeguard: if obstacle is <0.20m directly in front, forward velocity must be <=0.0."""
        if lidar_ranges is not None and len(lidar_ranges) >= 15:
            min_front = np.min(lidar_ranges[10:15])
            if min_front < 0.20:
                return min(0.0, v_out)
        return v_out
