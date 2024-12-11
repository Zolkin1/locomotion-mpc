import yaml

from acados_template import AcadosOcpCost


class CostSettings:
    def __init__(self, yaml_path: str):
        with open(yaml_path, 'r') as file:
            yaml_settings = yaml.safe_load(file)
            self.cost_list = yaml_settings["cost_params"]["cost_list"]

class Cost:
    def __init__(self, cost_settings: CostSettings):
        self._cost_settings = cost_settings

    def create_full_order_acados_cost(self) -> AcadosOcpCost:
        # TODO: Implement
        acados_cost = AcadosOcpCost()
        return acados_cost

    def create_centroidal_acados_cost(self) -> AcadosOcpCost:
        # TODO: Implement
        acados_cost = AcadosOcpCost()
        return acados_cost
