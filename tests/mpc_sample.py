import os
import numpy as np
from locomotion_mpc.LocomotionMPC import LocomotionMPC, create_mpc_from_yaml
from locomotion_mpc.utils import plot_pendulum

# print(os.environ["LD_LIBRARY_PATH"])
print(os.environ["ACADOS_SOURCE_DIR"])

# yaml_path = "../configurations/test_mpc.yaml"
yaml_path = "../configurations/cartpole_mpc.yaml"

# make an instance
mpc = create_mpc_from_yaml(yaml_path)

[status, traj] = mpc.solve_ocp()

print(traj.q_trajectory)

if status != 0 and status != 2:
    raise Exception(f'acados returned status {status}.')

# get solution
simX = np.zeros((mpc.settings.N+1, 4))
simU = np.zeros((mpc.settings.N, 1))

for i in range(mpc.settings.N):
    simX[i, :2] = traj.q_trajectory[i, :]
    simX[i, -2:] = traj.v_trajectory[i, :]

    simU[i, :] = traj.tau_trajectory[i, :]

simX[mpc.settings.N, :2] = traj.q_trajectory[mpc.settings.N, :]
simX[mpc.settings.N, -2:] = traj.v_trajectory[mpc.settings.N, :]

plot_pendulum(traj.time_traj, 10, simU, simX)