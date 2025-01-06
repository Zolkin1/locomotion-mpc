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
    def __init__(self):
        self.q_trajectory = []
        self.v_trajectory = []
        self.tau_trajectory = []
        self.F_trajectory = [[]]
        self.time_traj = []

    def get_config(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.q_trajectory[upper_idx-1],
                           self.q_trajectory[upper_idx], time)

    def get_vel(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.v_trajectory[upper_idx-1],
                           self.v_trajectory[upper_idx], time)

    def get_torque(self, time: float) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.v_trajectory[upper_idx-1],
                           self.v_trajectory[upper_idx], time)

    def get_force(self, time: float, frame_idx: int) -> np.array:
        upper_idx = self.get_upper_idx(time)
        t1 = self.time_traj[upper_idx - 1]
        t2 = self.time_traj[upper_idx]
        return interpolate(t1, t2, self.F_trajectory[frame_idx, upper_idx-1],
                           self.F_trajectory[frame_idx, upper_idx], time)

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
        latexify_plot()

        # plt.subplot(self. + 1, 1, 1)
        line, = plt.step(self.time_traj, self.tau_trajectory) #np.append([self.tau_trajectory[0, :]], self.tau_trajectory))
        # if X_true_label is not None:
        #     line.set_label(X_true_label)
        # else:
        #     line.set_color('r')
        # if title is not None:
        #     plt.title(title)
        plt.ylabel('$u$')
        plt.xlabel('$t$')
        # plt.hlines(u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
        # plt.hlines(-u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
        # plt.ylim([-1.2 * u_max, 1.2 * u_max])
        plt.grid()