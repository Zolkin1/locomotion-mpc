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

        self.config_lb = None
        self.config_ub = None
        self.vel_bounds = None
        self.torque_bounds = None

    def create_full_order_acados_constraints(self, acados_model: AcadosModel) -> AcadosOcpConstraints:
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
        acados_constraints.x0 = np.zeros((self.nx,))

        self.add_box_constraints(acados_constraints, True)

        # Friction cone/no force

        # Swing height

        # Holonomic

        # Collision

        # Contact polytope

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

        self.config_lb = robot_model.get_config_lb()
        self.config_ub = robot_model.get_config_ub()
        self.vel_bounds = robot_model.get_vel_bounds()
        self.torque_bounds = robot_model.get_torque_bounds()

        for vb in self.vel_bounds:
            if vb < 0:
                raise ValueError("Velocity bounds must be positive!")

        for tb in self.torque_bounds:
            if tb < 0:
                raise ValueError("Torque bounds must be positive!")

    def add_box_constraints(self, full_order: bool, acados_constraints: AcadosOcpConstraints):
        # TODO: Adjust to not limit floating base later
        if full_order:
            acados_constraints.idxbx = np.array(range(self.nx))
            acados_constraints.idxbu = np.array(range(self.nu))

            acados_constraints.lbx = np.vstack([self.config_lb, -self.vel_bounds])
            acados_constraints.ubx = np.vstack([self.config_ub, self.vel_bounds])

            acados_constraints.lbu = -self.torque_bounds
            acados_constraints.ubu = self.torque_bounds
        else:
            raise Exception("Not implemented!")

    def add_friction_constraints(self, acados_constraints: AcadosOcpConstraints):
        # Use casadi

    # TODO: Implement all the different constraints, have them be chosen in the yaml
    # TODO: Differentiate between foot frames and other contact frames
    # TODO: Use Acados parameters for the contact schedule
