import locomotion_mpc
from locomotion_mpc.LocomotionMPC import LocomotionMPC, create_mpc_from_yaml


yaml_path = "../configurations/test_mpc.yaml"

# make an instance
mpc = create_mpc_from_yaml(yaml_path)