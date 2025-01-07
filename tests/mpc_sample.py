import os
import numpy as np
import pinocchio as pin

from locomotion_mpc.LocomotionMPC import LocomotionMPC, create_mpc_from_yaml
from locomotion_mpc.utils import plot_pendulum

def plot_pend_traj(traj):
    # get solution
    simX = np.zeros((mpc.settings.N + 1, 4))
    simU = np.zeros((mpc.settings.N, 1))

    for i in range(mpc.settings.N):
        simX[i, :2] = traj.q_trajectory[i, :]
        simX[i, -2:] = traj.v_trajectory[i, :]

        simU[i, :] = traj.tau_trajectory[i, :]

    simX[mpc.settings.N, :2] = traj.q_trajectory[mpc.settings.N, :]
    simX[mpc.settings.N, -2:] = traj.v_trajectory[mpc.settings.N, :]

    plot_pendulum(traj.time_traj, 10, simU, simX)

# print(os.environ["LD_LIBRARY_PATH"])
print(os.environ["ACADOS_SOURCE_DIR"])

yaml_path = "../configurations/test_mpc.yaml"
# yaml_path = "../configurations/cartpole_mpc.yaml"

# make an instance
mpc = create_mpc_from_yaml(yaml_path)
robot_model = mpc.get_robot_model()

q = pin.neutral(robot_model.pin_model)
v = np.zeros((robot_model.nv,))

[status, traj] = mpc.solve_ocp(q, v)

if status != 0 and status != 2:
    raise Exception(f'acados returned status {status}.')

# plot_pend_traj(traj)

# Update the targets
qdes = pin.neutral(robot_model.pin_model)
# qdes[0] = 2
for i in range(mpc.settings.N):
    mpc.update_cost_targets(i, q_des=qdes)

# Change IC
# q[0] = -1

[status, traj] = mpc.solve_ocp(q, v)
traj.plot()

robot_model.create_visualizer()
robot_model.viz_trajectory(traj)

# plot_pend_traj(traj)
