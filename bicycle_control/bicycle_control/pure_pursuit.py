"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        ld = self.kv * abs(v) + self.l_min
        return float(np.clip(ld, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
        pts = np.asarray(path_points)
        dx = pts[:, 0] - x
        dy = pts[:, 1] - y
        dists = np.hypot(dx, dy)
        nearest_idx = int(np.argmin(dists))

        num_pts = len(pts)
        for i in range(num_pts):
            idx = (nearest_idx + i) % num_pts
            if dists[idx] >= lookahead:
                return idx, pts[idx] 
        
        fallback_idx = (nearest_idx + num_pts // 2) % num_pts
        return fallback_idx, pts[fallback_idx]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        dx = target_pt[0] - x
        dy = target_pt[1] - y
        y_local = -dx * math.sin(yaw) + dy * math.cos(yaw)
        ld = max(lookahead, 1e-3)
        alpha_steer = math.atan2(2.0 * self.L * y_local, ld ** 2)
        delta_rad = np.clip(alpha_steer, -self.max_steer_rad, self.max_steer_rad)
        return float(delta_rad)
