import math

import casadi
import numpy as np
import yaml
from acados_template import AcadosOcpConstraints, AcadosModel
from locomotion_mpc.robot_model.robot_model import RobotModel, FORCE_SIZE

class ConstraintSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.constraint_list = yaml_settings["constraint_params"]["constraint_list"]
            self.friction_coef = yaml_settings["constraint_params"]["friction_coef"]
            self.friction_eps = yaml_settings["constraint_params"]["friction_eps"]

class Constraints:
    def __init__(self, constraint_settings: ConstraintSettings):
        self._cost_settings = constraint_settings

        self.nq = None
        self.nv = None
        self.nx = None
        self.nu = None
        self.n_feet = None

        self.config_lb = None
        self.config_ub = None
        self.vel_bounds = None
        self.torque_bounds = None

    def create_full_order_acados_constraints(self, acados_model: AcadosModel) -> AcadosOcpConstraints:
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
        acados_constraints.x0 = np.zeros((self.nx,))

        self.add_box_constraints(True, acados_constraints)

        self.add_friction_constraints(True, acados_constraints, acados_model)

        # Swing height

        # Holonomic

        # Collision

        # Contact polytope

        print(acados_constraints.uh)
        print(acados_constraints.lh)
        print(acados_model.con_h_expr)

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
        self.nu = robot_model.nu
        self.nt = robot_model.full_order_torques
        self.nf = robot_model.nf

        self.n_feet = robot_model.nfeet

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
            acados_constraints.idxbu = np.array(range(self.nt))

            acados_constraints.lbx = np.vstack([self.config_lb, -self.vel_bounds])
            acados_constraints.ubx = np.vstack([self.config_ub, self.vel_bounds])

            acados_constraints.lbu = -self.torque_bounds
            acados_constraints.ubu = self.torque_bounds
        else:
            raise Exception("Not implemented!")


    def add_friction_constraints(self, full_order: bool, acados_constraints: AcadosOcpConstraints, acados_model: AcadosModel):
        # Use casadi
        F = acados_model.u[-self.nf:]

        for i in range(self.n_feet):
            # We have a different parameter vector for each node, so we just need to index by foot for the contact schedule
            # mu*Fz - sqrt(Fx^2 + Fy^2 + eps^2) >= 0
            cone = acados_model.p[i]*((self._cost_settings.friction_coef*F[2]
                    - casadi.sqrt(F[0]**2 + F[1]**2 + self._cost_settings.friction_eps**2)))
            acados_model.con_h_expr = casadi.vertcat(acados_model.con_h_expr, cone)

            if acados_constraints.lh.shape[0] > 0:
                acados_constraints.lh = np.vstack([acados_constraints.lh, 0])
                acados_constraints.uh = np.vstack([acados_constraints.uh, 1000])
            else:
                acados_constraints.lh = np.array([0])
                acados_constraints.uh = np.array([1000])

    # TODO: Implement all the different constraints, have them be chosen in the yaml
    # TODO: Differentiate between foot frames and other contact frames
    # TODO: Use Acados parameters for the contact schedule
