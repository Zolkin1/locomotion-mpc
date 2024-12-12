import numpy as np
import yaml
from acados_template import AcadosOcpConstraints, AcadosModel
from locomotion_mpc.robot_model.robot_model import RobotModel

class ConstraintSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.constraint_list = yaml_settings["constraint_params"]["constraint_list"]

class Constraints:
    def __init__(self, constraint_settings: ConstraintSettings):
        self._cost_settings = constraint_settings

        self.nq = None
        self.nv = None
        self.nx = None

        self.nu = None

    def create_full_order_acados_constraints(self, acados_model: AcadosModel) -> AcadosOcpConstraints:
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
        acados_constraints.x0 = np.zeros((self.nx,))
        return acados_constraints

    def create_full_order_acados_constraints_casadi(self, model: AcadosModel) -> AcadosModel:
        # TODO: Implement
        return model

    def create_centroidal_acados_constraints(self) -> AcadosOcpConstraints:
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
        return acados_constraints

    def create_centroidal_acados_constraints_casadi(self, model: AcadosModel):
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
        return acados_constraints

    def verify_sizes(self, robot_model: RobotModel):
        self.nq = robot_model.nq
        self.nv = robot_model.nv
        self.nx = self.nq + self.nv
        self.nu = robot_model.full_order_torques

    # TODO: Implement all the different constraints, have them be chosen in the yaml
    # TODO: Differentiate between foot frames and other contact frames
    # TODO: Use Acados parameters for the contact schedule
