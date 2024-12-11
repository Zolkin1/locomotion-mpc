
import yaml

from acados_template import AcadosOcp, AcadosOcpSolver
from kiwisolver import Constraint

from locomotion_mpc.robot_model.robot_model import RobotModel, RobotSettings
from locomotion_mpc.cost import Cost, CostSettings
from locomotion_mpc.constraints import Constraints, ConstraintSettings

class MpcSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.N = yaml_settings['mpc_params']['N']
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

        self._ocp = AcadosOcp()

        # Dynamics
        self._ocp.model = robot_model.create_acados_model(self._ocp.model)

        # Cost
        self._ocp.cost = cost.create_acados_cost()

        # Constraints
        self._ocp.constraints = constraints.create_acados_constraints()
        self._ocp.model = constraints.create_acados_constraints_casadi(self._ocp.model)

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

def create_mpc_from_yaml(yaml_path: str) -> LocomotionMPC:
    settings = MpcSettings(yaml_path)
    model_settings = RobotSettings(yaml_path)
    cost_settings = CostSettings(yaml_path)
    constraint_settings = ConstraintSettings(yaml_path)

    model = RobotModel(model_settings)
    cost = Cost(cost_settings)
    constraints = Constraints(constraint_settings)

    return LocomotionMPC(settings, model, cost, constraints)
