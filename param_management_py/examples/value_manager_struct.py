# Copyright 2024 Simon Sagmeister
from dataclasses import dataclass
import param_management_py as pmg


@dataclass(init=False)
class ModuleParameters:
    a: float
    b: float
    c: float


class SoftwareModule:

    def __init__(self):
        self.param_manager = pmg.ParamValueManager()
        self.params = ModuleParameters()
        self.param_state_hash = None
        self.declare_and_update_parameters()

    def declare_and_update_parameters(self):
        if self.param_manager.get_state_hash() != self.param_state_hash:
            # fmt: off
            self.params.a = self.param_manager.declare_and_get_value("a", 1.0, pmg.ParameterType.DOUBLE, "Description of a").as_double()  # noqa: E501
            self.params.b = self.param_manager.declare_and_get_value("b", 2.0, pmg.ParameterType.DOUBLE, "Description of b").as_double()  # noqa: E501
            self.params.c = self.param_manager.declare_and_get_value("c", 3.0, pmg.ParameterType.DOUBLE, "Description of c").as_double()  # noqa: E501
            # fmt: on
            self.param_state_hash = self.param_manager.get_state_hash()

    def step(self):
        return self.params.a * self.params.b * self.params.c


if __name__ == "__main__":
    mod = SoftwareModule()
    print(f"Result: {mod.step()}")
