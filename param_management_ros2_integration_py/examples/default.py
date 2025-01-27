# Copyright 2024 Simon Sagmeister
import rclpy
from rclpy.node import Node
from param_management_ros2_integration_py.helper_functions import (
    connect_param_manager_to_ros_cb,
    declare_ros_params_from_param_manager,
)
import param_management_py as pmg


class ROSIndependentSoftwareModule:

    def __init__(self):

        # This is an exemplary parameter that is only evaluated during creation of the node
        self.param_manager = pmg.ParamValueManager()
        self.value = None

        self._update_params()
        print("Value (Constructor): ", self.value)

        # Declare your parameters
        # CAREFUL: This will always return the default value here, since you
        # create the object before connecting it with the ros2 node

    def _update_params(self):
        self.value = self.param_manager.declare_and_get_value(
            "a", 3.14, pmg.ParameterType.DOUBLE, "A double parameter"
        ).as_double()

    def step(self):
        self._update_params()
        print("Value (Step): ", self.value)

    def get_param_manager(self):
        return self.param_manager


class ExampleNode(Node):

    def __init__(self, module: ROSIndependentSoftwareModule):
        super().__init__("TestNode")

        self.module = module

        connect_param_manager_to_ros_cb(
            self, self.module.get_param_manager()
        )  # Connect the param manager with the node -> Live updates
        declare_ros_params_from_param_manager(
            self, self.module.get_param_manager()
        )  # Declare all parameters from the param manager

        # From this point on, the module is parametrized correctly and is kept in sync
        self.module.step()


if __name__ == "__main__":
    rclpy.init()
    mod = ROSIndependentSoftwareModule()
    node = ExampleNode(mod)
    rclpy.spin(node)
    rclpy.shutdown()
