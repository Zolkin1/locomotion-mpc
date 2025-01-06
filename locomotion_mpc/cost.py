import numpy as np
import pinocchio
import yaml
import casadi
from acados_template import AcadosOcpCost, AcadosModel
from locomotion_mpc.robot_model.robot_model import RobotModel, FLOATING_VEL, GRAV

class CostSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.cost_types = []
            self.cost_weights = []
            for cost_terms in yaml_settings['cost_params']:
                self.cost_types.append(cost_terms['cost_type'])
                self.cost_weights.append(cost_terms['weights'])

class Cost:
    def __init__(self, cost_settings: CostSettings):
        self._cost_settings = cost_settings

        self._nq = None
        self._nv = None
        self._ntorque = None
        self._nf = 3
        self.nfoot_frames = None
        self._robot_mass = None

        self._cost_idx = 0

        self._all_costs_types = ["torque_reg", "force_reg", "velocity_tracking", "config_tracking"]
        self._cost_sizes = None

    def create_full_order_acados_cost(self, acados_model: AcadosModel) -> AcadosOcpCost:
        acados_cost = AcadosOcpCost()

        nx = self._nq + self._nv
        nu = self._ntorque + self._nf*self.nfoot_frames


        w_size = 0
        for i in range(len(self._all_costs_types)):
            cost_type = self._all_costs_types[i]
            if cost_type in self._cost_settings.cost_types:
                w_size += self._cost_sizes[i]

        acados_cost.cost_type = "NONLINEAR_LS"
        acados_cost.W = np.zeros((w_size, w_size))
        acados_cost.yref = np.zeros((w_size,))

        self.add_regularization(True, acados_cost, acados_model)
        self.add_tracking(True, acados_cost, acados_model)

        self._cost_idx = 0

        return acados_cost

    def create_centroidal_acados_cost(self, acados_model: AcadosModel) -> AcadosOcpCost:
        acados_cost = AcadosOcpCost()

        nx = self._nq + FLOATING_VEL
        nu = self._nv - FLOATING_VEL + self._nf*self.nfoot_frames

        w_size = 0
        for i in range(len(self._all_costs_types)):
            cost_type = self._all_costs_types[i]
            if cost_type in self._cost_settings.cost_types:
                w_size += self._cost_sizes[i]

        acados_cost.cost_type = "LINEAR_LS"
        acados_cost.W = np.zeros((w_size, w_size))
        acados_cost.yref = np.zeros((w_size,))

        self.add_regularization(False, acados_cost, acados_model)
        self.add_tracking(False, acados_cost, acados_model)

        self._cost_idx = 0

        return acados_cost

    def add_regularization(self, full_order: bool, acados_cost: AcadosOcpCost, acados_model: AcadosModel):
        if "torque_reg" in self._cost_settings.cost_types and full_order:
            weights = self._cost_settings.cost_weights[self._cost_settings.cost_types.index('torque_reg')]
            acados_cost.W[self._cost_idx:self._cost_idx + self._ntorque, self._cost_idx:self._cost_idx + self._ntorque] = np.diag(weights)
            acados_cost.yref[self._cost_idx:self._cost_idx + self._ntorque] = np.zeros((self._ntorque,))
            acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.u[:self._ntorque])
            self._cost_idx += self._ntorque

        if "force_reg" in self._cost_settings.cost_types:
            weights = self._cost_settings.cost_weights[self._cost_settings.cost_types.index('force_reg')]
            vu_idx = FLOATING_VEL
            if full_order:
                vu_idx = self._ntorque
            for i in range(self.nfoot_frames):
                acados_cost.W[self._cost_idx:self._cost_idx + self._nf, self._cost_idx:self._cost_idx + self._nf] = np.diag(weights)
                # TODO: Make this desired force based on the parameters (contacts)
                acados_cost.yref[self._cost_idx:self._cost_idx + self._nf] = [0.0, 0.0, GRAV*self._robot_mass/self.nfoot_frames]
                acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.u[vu_idx:vu_idx + self._nf])
                vu_idx += self._nf
                self._cost_idx += self._nf

    # TODO: Add the parameters for tracking
    def add_tracking(self, full_order: bool, acados_cost: AcadosOcpCost, acados_model: AcadosModel):
        if "velocity_tracking" in self._cost_settings.cost_types:
            weights = self._cost_settings.cost_weights[self._cost_settings.cost_types.index('velocity_tracking')]
            if full_order:
                acados_cost.W[self._cost_idx:self._cost_idx + self._nv, self._cost_idx:self._cost_idx + self._nv] = np.diag(weights)
                # self.assign_casadi_to_list(acados_model.cost_y_expr,casadi.vertsplit(acados_model.x[self._nq:self._nq + self._nv], 1))
                acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.x[self._nq:self._nq + self._nv])
                self._cost_idx += self._nv
            else:
                acados_cost.W[self._cost_idx:self._cost_idx + self._nv, self._cost_idx:self._cost_idx + self._nv] = np.diag(weights)
                self._cost_idx += self._nv
                acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.x[self._nq:self._nq + FLOATING_VEL])
                acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.u[:FLOATING_VEL])


        if "config_tracking" in self._cost_settings.cost_types:
            weights = self._cost_settings.cost_weights[self._cost_settings.cost_types.index('config_tracking')]
            acados_cost.W[self._cost_idx: self._cost_idx + self._nq, self._cost_idx:self._cost_idx+self._nq] = np.diag(weights)
            acados_model.cost_y_expr = casadi.vertcat(acados_model.cost_y_expr, acados_model.x[:self._nq])
            acados_cost.yref[self._cost_idx:self._cost_idx + self._nq] = np.array([1, 0])
            self._cost_idx += self._nq

    def assign_casadi_to_list(self, casadi_expr_list, casadi_expr):
        for expr in casadi_expr:
            casadi_expr_list.append(expr)

    def verify_sizes(self, robot_model: RobotModel):
        if "torque_reg" in self._cost_settings.cost_types:
            if len(self._cost_settings.cost_weights[self._cost_settings.cost_types.index('torque_reg')]) != robot_model.ntau:
                raise ValueError("[Cost] Torque weights do not match robot model!")
        self._ntorque = robot_model.ntau

        if "force_reg" in self._cost_settings.cost_types:
            if len(self._cost_settings.cost_weights[self._cost_settings.cost_types.index('force_reg')]) != 3:
                raise ValueError("[Cost] Force weights do not match robot model!")
        self.nfoot_frames = len(robot_model._settings.foot_frames)
        self._robot_mass = 1 #pinocchio.computeTotalMass(robot_model.pin_model)

        if "velocity_tracking" in self._cost_settings.cost_types:
            if len(self._cost_settings.cost_weights[self._cost_settings.cost_types.index('velocity_tracking')]) != robot_model.nv:
                raise ValueError("[Cost] Velocity tracking weights do not match robot model!")
        self._nv = robot_model.nv

        if "config_tracking" in self._cost_settings.cost_types:
            if len(self._cost_settings.cost_weights[self._cost_settings.cost_types.index('config_tracking')]) != robot_model.nq:
                raise ValueError("[Cost] Configuration tracking weights do not match robot model!")
        self._nq = robot_model.nq

        self._cost_sizes = [self._ntorque, self._nf*self.nfoot_frames, self._nv, self._nq]

    def get_yref_size(self):
        w_size = 0
        for i in range(len(self._all_costs_types)):
            cost_type = self._all_costs_types[i]
            if cost_type in self._cost_settings.cost_types:
                w_size += self._cost_sizes[i]

        return w_size

    def print(self):
        print("MPC Cost:")
        for i in range(len(self._cost_settings.cost_types)):
            print(f"\tType: {self._cost_settings.cost_types[i]}")
            print(f"\tWeight: {self._cost_settings.cost_weights[i]}")
            if i < len(self._cost_settings.cost_types) - 1:
                print("\t---------")