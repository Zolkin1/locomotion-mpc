import numpy as np
import yaml
import pinocchio
import pinocchio.casadi as cpin
import casadi
from casadi import SX, vertcat
from acados_template import AcadosModel

class RobotSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.urdf_path = yaml_settings["model_params"]["urdf_path"]
            self.contact_frames = yaml_settings["model_params"]["contact_frames"]

class RobotModel:
    def __init__(self, settings: RobotSettings) -> None:
        # Create the model
        self.pin_model = pinocchio.buildModelFromUrdf(settings.urdf_path)
        self.cpin_model = cpin.Model(self.pin_model)

        # Create the data
        self.pin_data = self.pin_model.createData()
        self.cpin_data = self.cpin_model.createData()

        print("model name: " + self.pin_model.name)

        self._settings = settings

        self.create_forward_dynamics()

    def create_acados_model(self, model: AcadosModel) -> AcadosModel:
        # TODO: Finish
        x = vertcat(self.q_node, self.v_node)
        u = self.u_node
        xdot = SX.sym("xdot")

        # TODO: Check
        f_expl = self.fd_func(x[:self.pin_model.nq], x[self.pin_model.nv:], u)
        f_impl = xdot - f_expl

        # TODO: Print/test that this is doing what I want
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = self.pin_model.name

        return model    # TODO: Do I need to return this, or is it modified in place (i.e. like a pass by reference)

    def create_forward_dynamics(self):
        """
        Create a function giving the forward dynamics.

        Need to convert the forces into the proper frames then calls the forward dynamics function.
        """
        nq = self.pin_model.nq
        nu = self.pin_model.nv
        nv = self.pin_model.nv
        q = SX.sym("q", nq)
        v = SX.sym("v", nv)
        u = SX.sym("u", nu)
        dq_ = SX.sym("dq_", nv)

        # TODO: Change these names
        self.u_node = u
        self.q_node = q
        self.v_node = v
        self.dq_ = dq_

        # TODO: Add in the external forces and their frame change
        # TODO: Deal with the quaternion properly
        # TODO: Deal with no torques on the floating base (i.e. u needs to be smaller)

        tau = u # TODO: Change
        a = cpin.aba(self.cpin_model, self.cpin_data, q, v, tau)
        self.fd_func = casadi.Function("acc", [q, v, u], [a], ["q", "v", "u"], ["a"])
