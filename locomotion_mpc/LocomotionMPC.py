
import numpy as np
import yaml

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosMultiphaseOcp, AcadosModel
from casadi import SX, vertcat

from locomotion_mpc.robot_model.robot_model import RobotModel, RobotSettings
from locomotion_mpc.cost import Cost, CostSettings
from locomotion_mpc.constraints import Constraints, ConstraintSettings

class MpcSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.N = yaml_settings['mpc_params']['N']
            self.N_full_order = yaml_settings['mpc_params']['N_full_order']
            self.Tf = yaml_settings['mpc_params']['Tf']
            self.qp_solver = yaml_settings['mpc_params']['qp_solver']
            self.max_iter = yaml_settings['mpc_params']['max_iter']
            self.hessian_approx = yaml_settings['mpc_params']['hessian_approx']
            self.integrator_type = yaml_settings['mpc_params']['integrator_type']
            self.print_level = yaml_settings['mpc_params']['print_level']

    def print(self):
        print("MPC Params:")
        print(f"\tN: {self.N}")
        print(f"\tTf: {self.Tf}")
        print(f"\tqp_solver: {self.qp_solver}")
        print(f"\tmax_iter: {self.max_iter}")

class LocomotionMPC:
    """
    Locomotion MPC class designed to create and manage an AcadosOCP object, the robot model, and the constraints
    """
    def __init__(self, mpc_settings: MpcSettings, robot_model: RobotModel, cost: Cost, constraints: Constraints) -> None:
        self._settings = mpc_settings
        self._settings.print()

        self._robot_model = robot_model
        self._cost = cost
        self._constraints = constraints

        self._robot_model.print()

        N_list = [self._settings.N_full_order, 1, self._settings.N - self._settings.N_full_order]
        self._ocp = AcadosMultiphaseOcp(N_list)

        # Full order MPC phase
        full_order = self.CreateFullOrderOCP()
        self._ocp.set_phase(full_order, 0)

        # Transition
        transition = self.CreateTransitionOCP()
        self._ocp.set_phase(transition, 1)

        # Centroidal
        centroidal = self.CreateCentroidalOCP()
        self._ocp.set_phase(centroidal, 2)

        # Solver Settings
        self.assign_settings()

        # Create the solver
        # self._ocp_solver = AcadosOcpSolver(self._ocp)

    def assign_settings(self):
        self._ocp.solver_options.N_horizon = self._settings.N
        self._ocp.solver_options.Tsim = self._settings.Tf

        self._ocp.solver_options.max_iter = self._settings.max_iter
        self._ocp.solver_options.hessian_approx = self._settings.hessian_approx
        self._ocp.solver_options.integrator_type = self._settings.integrator_type
        self._ocp.solver_options.print_level = self._settings.print_level

        if self._settings.qp_solver == 'OSQP':
            self._ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_OSQP"
        elif self._settings.qp_solver == 'QPOASES':
            self._ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
        elif self._settings.qp_solver == 'QPDUNES':
            self._ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_QPDUNES"
        elif self._settings.qp_solver == 'DAQP':
            self._ocp.solver_options.qp_solver = "FULL_CONDENSING_DAQP"
        elif self._settings.qp_solver == 'FULL_HPIPM':
            self._ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        elif self._settings.qp_solver == 'PARTIAL_HPIPM':
            self._ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        else:
            raise Exception("Invalid QP solver!")

    # TODO: Add a function to update the cost target parameters
    # TODO: Add a function to update the contact schedule parameters
    # TODO: Add a function to solve from the current state (passed in)
    # TODO: Add a function to plot the COM traj to start to verify it
    # TODO: Formulate the multi-model problem using the multi-phase OCP in Acados (https://docs.acados.org/python_interface/index.html#acados-multi-phase-ocp)
    # TODO: Try using pinocchio with the meshcat viewer to visualize the results

    def CreateFullOrderOCP(self) -> AcadosOcp:
        """Create the OCP for the full order dynamics model."""
        ocp = AcadosOcp()

        # Dynamics
        ocp.model = self._robot_model.create_full_order_acados_model(ocp.model)

        # Cost
        ocp.cost = self._cost.create_full_order_acados_cost()

        # Constraints
        ocp.constraints = self._constraints.create_full_order_acados_constraints()
        ocp.model = self._constraints.create_full_order_acados_constraints_casadi(ocp.model)

        return ocp

    def CreateTransitionOCP(self) -> AcadosOcp:
        """Create the OCP for the transition stage."""
        model = AcadosModel()

        model.name = "transition_model"

        q = SX.sym('q', self._robot_model.pin_model.nq)
        v = SX.sym('v', self._robot_model.pin_model.nv)
        model.x = vertcat(q, v)
        x_size = model.x.size()[0]

        model.disc_dyn_expr = vertcat(q, v[:6])     # The centroidal model uses the joint velocities as inputs, not states

        # Create the OCP
        ocp = AcadosOcp()

        ocp.model = model

        # TODO: Not entirely sure what I should use for the cost
        #   For now making the cost have 0's
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = ocp.model.x
        ocp.cost.W = np.zeros((x_size, x_size))
        ocp.cost.yref = np.zeros((x_size, 1))

        return ocp

    def CreateCentroidalOCP(self) -> AcadosOcp:
        """Create the OCP for the centroidal dynamics model."""
        ocp = AcadosOcp()

        # Dynamics
        ocp.model = self._robot_model.create_centroidal_acados_model(ocp.model)

        # Cost
        ocp.cost = self._cost.create_centroidal_acados_cost()

        # Constraints
        ocp.constraints = self._constraints.create_centroidal_acados_constraints()
        ocp.model = self._constraints.create_centroidal_acados_constraints_casadi(ocp.model)

        return ocp

def create_mpc_from_yaml(yaml_path: str) -> LocomotionMPC:
    settings = MpcSettings(yaml_path)
    model_settings = RobotSettings(yaml_path)
    cost_settings = CostSettings(yaml_path)
    constraint_settings = ConstraintSettings(yaml_path)

    model = RobotModel(model_settings)
    cost = Cost(cost_settings)
    constraints = Constraints(constraint_settings)

    return LocomotionMPC(settings, model, cost, constraints)
