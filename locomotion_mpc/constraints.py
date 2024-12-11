import yaml
from acados_template import AcadosOcpConstraints, AcadosModel

class ConstraintSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.constraint_list = yaml_settings["constraint_params"]["constraint_list"]

class Constraints:
    def __init__(self, constraint_settings: ConstraintSettings):
        self._cost_settings = constraint_settings

    def create_full_order_acados_constraints(self) -> AcadosOcpConstraints:
        # TODO: Implement
        acados_constraints = AcadosOcpConstraints()
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

    # TODO: Implement all the different constraints, have them be chosen in the yaml
    # TODO: Differentiate between foot frames and other contact frames
    # TODO: Use Acados parameters for the contact schedule
