import numpy as np
import yaml
import pinocchio
import pinocchio.casadi as cpin
import casadi
from casadi import SX, vertcat
from acados_template import AcadosModel

from locomotion_mpc.robot_model.rotation_transformations import *

# FLOATING_BASE = 7
FLOATING_VEL = 6
FORCE_SIZE = 3

class RobotSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.urdf_path = yaml_settings["model_params"]["urdf_path"]
            self.foot_frames = yaml_settings["model_params"]["foot_frames"]
            self.locked_joints = yaml_settings["model_params"]["locked_joints"]
            self.locked_defaults = yaml_settings["model_params"]["locked_defaults"]

            if (len(self.locked_joints) != len(self.locked_defaults)):
                raise ValueError("Size of locked joints must match the number of default lock positions!")

class RobotModel:
    """
    Robot Model class used for MPC.

    Note:
        - Euler angles are currently being used because I can't figure out how to use quaternions properly in Acados
        - All velocities are local to that frame, including the floating base
    """
    def __init__(self, settings: RobotSettings) -> None:
        self._settings = settings

        # Create an euler angle root joint
        base_joint = pinocchio.JointModelComposite()
        base_joint.addJoint(pinocchio.JointModelTranslation())
        base_joint.addJoint(pinocchio.JointModelSphericalZYX())

        # Create the full model
        full_pin_model = pinocchio.buildModelFromUrdf(settings.urdf_path, base_joint)

        # Lock certain joints
        q_default = pinocchio.neutral(full_pin_model)
        locked_joint_ids = []
        for i in range(len(self._settings.locked_joints)):
            joint = self._settings.locked_joints[i]
            if full_pin_model.existJointName(joint):
                locked_joint_ids.append(full_pin_model.getJointId(joint))
                q_default[full_pin_model.getJointId(joint)] = self._settings.locked_defaults[i]

        # Create pinocchio models
        self.pin_model = pinocchio.buildReducedModel(full_pin_model, locked_joint_ids, q_default)
        self.cpin_model = cpin.Model(self.pin_model)

        # Create the data
        self.pin_data = self.pin_model.createData()
        self.cpin_data = self.cpin_model.createData()

        self.verify_frames()

        # ------------------------------- #
        # ----- Full Order Dynamics ----- #
        # ------------------------------- #
        nq = self.pin_model.nq
        nv = self.pin_model.nv
        self.nf = FORCE_SIZE * len(self._settings.foot_frames)
        self.full_order_torques = self.pin_model.nv - FLOATING_VEL   # Fully actuated except for floating base

        q = SX.sym("q", nq)
        v = SX.sym("v", nv)
        u = SX.sym("u", self.full_order_torques)
        F = SX.sym("F", self.nf)

        self.u_full_order = u
        self.q_full_order = q
        self.v_full_order = v
        self.F_full_order = F
        # ------------------------------- #
        # ------------------------------- #
        # ------------------------------- #

        self.create_forward_dynamics()
        self.create_integration_dynamics()


    def create_full_order_acados_model(self, model: AcadosModel) -> AcadosModel:
        """Create an Acados model using the full order ROM."""
        # States
        x = vertcat(self.q_full_order, self.v_full_order)

        # Inputs
        u = vertcat(self.u_full_order, self.F_full_order)

        # xdot
        xdot = SX.sym("xdot_fo", self.pin_model.nq + self.pin_model.nv)

        # Dynamics function
        # TODO: Check
        f_expl = vertcat(self.vel_dyn_func(self.q_full_order, self.v_full_order),
                         self.fd_func(self.q_full_order, self.v_full_order, self.u_full_order, self.F_full_order))

        f_impl = xdot - f_expl

        # Assign to model
        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = self.pin_model.name + "_full_order"

        return model    # TODO: Do I need to return this, or is it modified in place (i.e. like a pass by reference)

    def create_centroidal_acados_model(self, model: AcadosModel) -> AcadosModel:
        """Create an Acados model using the centroidal ROM."""
        # States
        x = vertcat(self.q_full_order, self.v_full_order[:FLOATING_VEL])

        # Inputs
        u = vertcat(self.v_full_order[FLOATING_VEL:], self.F_full_order)

        # xdot
        xdot = SX.sym("xdot_centroidal", self.pin_model.nq + FLOATING_VEL)

        # Dynamics function
        f_expl = vertcat(self.vel_dyn_func(self.q_full_order, self.v_full_order),
                         self.fd_func(self.q_full_order, self.v_full_order, self.u_full_order, self.F_full_order)[:FLOATING_VEL])

        f_impl = xdot - f_expl

        # Assign to model
        model.f_expl_expr = f_expl
        model.f_impl_expr = f_impl
        model.x = x
        model.u = u
        model.xdot = xdot
        model.name = self.pin_model.name + "_centroidal"

        return model

    def create_forward_dynamics(self):
        """
        Create a function giving the forward dynamics.

        Need to convert the forces into the proper frames then calls the forward dynamics function.
        """
        floating_base_torque = SX.zeros(FLOATING_VEL, 1)
        tau = vertcat(floating_base_torque, self.u_full_order)

        f_ext = self.get_local_external_forces(self.q_full_order, self.F_full_order)

        # TODO: Confirm this claim:
        # From looking at Featherstone, I believe that v is the angular velocities, not the euler angle derivatives
        # Forward dynamics
        a = cpin.aba(self.cpin_model, self.cpin_data, self.q_full_order, self.v_full_order, tau, f_ext)

        # Make casadi function
        self.fd_func = casadi.Function("acc", [self.q_full_order, self.v_full_order, self.u_full_order, self.F_full_order],
                                       [a], ["q", "v", "u", "F"], ["a"])

    def create_integration_dynamics(self):
        """Create a casadi function giving the integration dynamics."""
        T = get_map_from_euler_zyx_deriv_to_local_angular_vel(self.q_full_order[3:6])
        Tinv = np.linalg.inv(T)

        dq = vertcat(self.v_full_order[:3],
                     Tinv @ self.v_full_order[3:6],
                     self.v_full_order[6:])

        self.vel_dyn_func = casadi.Function("vel_dynamics", [self.q_full_order, self.v_full_order], [dq], ["q", "v"], ["dq"])

    def get_local_external_forces(self, q: SX, F: SX) -> list:
        """Convert the forces F to the corresponding joint frame given by the foot frames."""
        f_ext = cpin.StdVec_Force(size=self.pin_model.njoints, value=cpin.Force.Zero())

        cpin.framesForwardKinematics(self.cpin_model, self.cpin_data, q)

        F_idx = 0
        for frame in self._settings.foot_frames:
            # Get contact frame
            frame_idx = self.pin_model.getFrameId(frame)

            # Get parent frame
            jnt_idx = self.pin_model.frames[frame_idx].parentJoint

            # Translation from joint frame to contact frame
            transl_contact_to_joint = self.pin_model.frames[frame_idx].placement.translation

            # Rotation from world frame to joint frame
            rotation_world_joint = self.pin_data.oMi[jnt_idx].rotation.transpose()

            # Contact forces in the joint frame
            contact_force = rotation_world_joint @ F[F_idx:F_idx+FORCE_SIZE]

            f_ext[jnt_idx].linear += contact_force

            # Angular forces (torques)
            f_ext[jnt_idx].angular += casadi.cross(transl_contact_to_joint, contact_force)

            F_idx += FORCE_SIZE

        return f_ext

    def verify_frames(self):
        for frame in self._settings.foot_frames:
            if self.pin_model.getFrameId(frame) == len(self.pin_model.frames):
                raise ValueError("Frame " + str(frame) + " is not found in the URDF!")

    def print(self):
        print("Robot Model:")
        print(f"\tnq: {self.pin_model.nq}")
        print(f"\tnv: {self.pin_model.nv}")
        print(f"\ttorques: {self.full_order_torques}")
        print(f"\tnf: {self.nf}")
        print(f"\tfoot frames: {self._settings.foot_frames}")

    # TODO: Implement inverse dynamics, which I think will need to use the implicit formulation or the explicit discrete