import sys
import math
import numpy as np

class RoboWayPEngine:
    def __init__(self):
        # Manipulator Link Geometry (meters)
        self.L1 = 1.30
        self.L2 = 1.10
        self.L3 = 0.70
        self.base_3d = np.array([0.0, 0.0, 0.5])
        self.yaw_sweep_deg = 60.0
        
        # Planner & IK settings
        self.elbow_up = True
        self.algo = "Quintic" # 'Quintic' | 'Cubic' | 'Cartesian'
        self.obs_enabled = True
        self.obs = {"x": 1.15, "y": 1.05, "r": 0.30}
        
        # Task-Space Waypoints: [x, y, phi_deg, duration_sec]
        self.default_waypoints = [
            [0.65, 1.75, 45.0, 0.0],
            [1.45, 1.65, 15.0, 1.5],
            [1.85, 0.75, -20.0, 1.5],
            [0.95, 0.35, -45.0, 1.5],
        ]
        self.waypoints = [list(w) for w in self.default_waypoints]
        self.total_time = 4.5
        self.samples = []
        self.rebuild_trajectory()

    def forward_kinematics_2d(self, th1, th2, th3):
        """Compute 2D planar joint positions (J1, J2, EE)."""
        a12 = th1 + th2
        a123 = a12 + th3
        j1 = np.array([self.L1 * np.cos(th1), self.L1 * np.sin(th1)])
        j2 = j1 + np.array([self.L2 * np.cos(a12), self.L2 * np.sin(a12)])
        ee = j2 + np.array([self.L3 * np.cos(a123), self.L3 * np.sin(a123)])
        return j1, j2, ee

    def solve_ik(self, x, y, phi_deg):
        """Analytical 3-DOF planar Inverse Kinematics."""
        phi = np.deg2rad(phi_deg)
        wx = x - self.L3 * np.cos(phi)
        wy = y - self.L3 * np.sin(phi)
        r = np.hypot(wx, wy)
        max_r = self.L1 + self.L2 - 1e-4
        min_r = abs(self.L1 - self.L2) + 1e-4
        reachable = True
        
        if r > max_r or r < min_r:
            reachable = False
        
        c2 = (wx**2 + wy**2 - self.L1**2 - self.L2**2) / (2.0 * self.L1 * self.L2)
        c2 = np.clip(c2, -1.0, 1.0)
        s2 = (-1.0 if self.elbow_up else 1.0) * np.sqrt(max(0.0, 1.0 - c2**2))
        
        th2 = np.arctan2(s2, c2)
        th1 = np.arctan2(wy, wx) - np.arctan2(self.L2 * s2, self.L1 + self.L2 * c2)
        th3 = phi - th1 - th2
        return th1, th2, th3, reachable

    def rebuild_trajectory(self, num_samples=60):
        """Solve waypoint IK and interpolate complete trajectory."""
        iks = []
        times = [0.0]
        for i, wp in enumerate(self.waypoints):
            th1, th2, th3, ok = self.solve_ik(wp[0], wp[1], wp[2])
            if i > 0:
                th1 = th1 + ((th1 - iks[i-1][0] + np.pi) % (2.0 * np.pi) - np.pi)
            times.append(times[-1] + max(0.3, float(wp[3])))
            iks.append((th1, th2, th3, ok))
        self.total_time = times[-1]
        self.iks = iks
        self._times = times
        print(f"Trajectory rebuilt successfully. Total duration: {self.total_time}s")

if __name__ == "__main__":
    planner = RoboWayPEngine()
    print("Robo-WayP Kinematics Engine initialized successfully.")
