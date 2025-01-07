import numpy as np
import matplotlib.pyplot as plt
from acados_template import latexify_plot

def interpolate(t1, t2, val1, val2, t):
    if t > t2 or t < t1:
        print(f"t1: {t1}, t2: {t2}, val1: {val1}, val2: {val2}")
        raise ValueError("Interpolation time is invalid!")

    lam = (t - t1)/(t2 - t1)
    return (1-lam)*val1 + lam*val2


class Trajectory:
    def __init__(self, nq, nv, ntau, nf, nt):
        self.q_trajectory = np.zeros((nt+1, nq))
        self.v_trajectory = np.zeros((nt+1, nv))
        self.tau_trajectory = np.zeros((nt, ntau))
        self.F_trajectory = np.zeros((nt, nf))
        self.time_traj = np.zeros((nt+1,))

    def get_config(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.q_trajectory[upper_idx-1,:],
                           self.q_trajectory[upper_idx,:], time)

    def get_vel(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.v_trajectory[upper_idx-1, :],
                           self.v_trajectory[upper_idx, :], time)

    def get_torque(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.tau_trajectory[upper_idx-1, :],
                           self.tau_trajectory[upper_idx, :], time)

    def get_force(self, time: float, frame_idx: int) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.F_trajectory[upper_idx-1, 3*frame_idx: 3*frame_idx+3],
                           self.F_trajectory[upper_idx, 3*frame_idx: 3*frame_idx+3], time)

    def get_upper_idx(self, time: float) -> int:
        if time == self.time_traj[-1]:
            return len(self.time_traj)-1

        i = 0
        while time >= self.time_traj[i]:
            i += 1

        assert i > 0

        return i

    def print(self):
        print(f"q: {self.q_trajectory}")
        print(f"v: {self.v_trajectory}")
        print(f"tau: {self.tau_trajectory}")
        print(f"F: {self.F_trajectory}")
        print(f"times: {self.time_traj}")

    def plot(self):
        # 3 subplots for the configuration in general: position, orientation, joints
        # Same 3 subplots for the velocities
        # Each grouping of 3 force scalars gets a subplot
        # Torques are all on the same subplot

        latexify_plot()

        # ---- Inputs ----- #
        plt.subplot(5, 1, 1)
        for i in range(self.tau_trajectory.shape[1]):
            u_plot_traj = self.tau_trajectory[:,i]
            line, = plt.step(self.time_traj, np.append(self.tau_trajectory[0,i], u_plot_traj))
        plt.ylabel('$u$')
        plt.xlabel('$t$')
        plt.grid()

        # ----- Joint Angles ----- #
        plt.subplot(5, 1, 2)
        for i in range(6, self.q_trajectory.shape[1]):
            q_plot_traj = self.q_trajectory[:,i]
            line, = plt.plot(self.time_traj, q_plot_traj)

        plt.ylabel('$q$ joints')
        plt.xlabel('$t$')
        plt.grid()

        # ----- Joint Velocities ---- #
        plt.subplot(5, 1, 3)
        for i in range(6, self.v_trajectory.shape[1]):
            v_plot_traj = self.v_trajectory[:,i]
            line, = plt.plot(self.time_traj, v_plot_traj)

        plt.ylabel('$v$ joints')
        plt.xlabel('$t$')
        plt.grid()

        # ----- Floating Base Position ----- #
        plt.subplot(5, 1, 4)
        for i in range(6):
            q_plot_traj = self.q_trajectory[:,i]
            line, = plt.plot(self.time_traj, q_plot_traj)

        plt.ylabel('$q$ base')
        plt.xlabel('$t$')
        plt.grid()

        # ----- Floating Base Velocities ---- #
        plt.subplot(5, 1, 5)
        for i in range(6):
            v_plot_traj = self.v_trajectory[:,i]
            line, = plt.plot(self.time_traj, v_plot_traj)

        plt.ylabel('$v$ base')
        plt.xlabel('$t$')
        plt.grid()

        plt.show()