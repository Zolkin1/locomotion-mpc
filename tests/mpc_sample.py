import os
import numpy as np
from locomotion_mpc.LocomotionMPC import LocomotionMPC, create_mpc_from_yaml
from locomotion_mpc.utils import plot_pendulum

# print(os.environ["LD_LIBRARY_PATH"])
print(os.environ["ACADOS_SOURCE_DIR"])

yaml_path = "../configurations/test_mpc.yaml"
# yaml_path = "../configurations/cartpole_mpc.yaml"

# make an instance
mpc = create_mpc_from_yaml(yaml_path)

status = mpc.solve_ocp()

if status != 0 and status != 2:
    raise Exception(f'acados returned status {status}.')

robot_model = mpc.get_robot_model()

# get solution
simX = np.zeros((mpc.settings.N+1, robot_model.nq + robot_model.nv))
simU = np.zeros((mpc.settings.N, robot_model.nu))

for i in range(mpc.settings.N):
    simX[i, :] = mpc.ocp_solver.get(i, "x")
    # print(mpc.ocp_solver.get(i, "x"))
    simU[i, :] = mpc.ocp_solver.get(i, "u")[0]
    # print(mpc.ocp_solver.get(i, "u"))
simX[mpc.settings.N, :] = mpc.ocp_solver.get(mpc.settings.N, "x")

# plot_trajectory(np.linspace(0, mpc.settings.Tf, mpc.settings.N + 1), 10, simU, simX)

# plot_pendulum(np.linspace(0, mpc.settings.Tf, mpc.settings.N + 1), 10, simU, simX)

# TODO: Plot the values for generic robots