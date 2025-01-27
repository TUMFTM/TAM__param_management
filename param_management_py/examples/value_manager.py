# Copyright 2024 Simon Sagmeister
import param_management_py as pmg


class SoftwareModule:

    def __init__(self):
        self.param_manager = pmg.ParamValueManager()
        self.declare_parameters()

    def declare_parameters(self):
        # fmt: off
        self.param_manager.declare_parameter("a", 1.0, pmg.ParameterType.DOUBLE, "Description of a")  # noqa: E501
        self.param_manager.declare_parameter("b", 2.0, pmg.ParameterType.DOUBLE, "Description of b")  # noqa: E501
        self.param_manager.declare_parameter("c", 3.0, pmg.ParameterType.DOUBLE, "Description of c")  # noqa: E501
        # fmt: on

    def step(self):

        return (
            self.param_manager.get_value("a").as_double()
            * self.param_manager.get_value("b").as_double()
            * self.param_manager.get_value("c").as_double()
        )


if __name__ == "__main__":
    mod = SoftwareModule()
    print(f"Result: {mod.step()}")
