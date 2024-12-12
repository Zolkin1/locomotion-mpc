import os
from locomotion_mpc.LocomotionMPC import LocomotionMPC, create_mpc_from_yaml

print(os.environ["LD_LIBRARY_PATH"])
print(os.environ["ACADOS_SOURCE_DIR"])

yaml_path = "../configurations/test_mpc.yaml"

# make an instance
mpc = create_mpc_from_yaml(yaml_path)

mpc.solve_ocp()