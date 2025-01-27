# Copyright 2024 Simon Sagmeister
import rclpy
from rclpy.node import Node
from param_management_ros2_integration_py.rclpy_value_manager import RCLPYValueManager
import param_management_py as pmg

# This is a useful implementation if you need your parameters already with the correct overwrite values
# while declaring your parameter. This can be helpful is a specific setup during initialization of your
# software module is done only once in the constructor


class ROSIndependentSoftwareModule:

    def __init__(self, param_manager: pmg.UserInterface):

        # This is an exemplary parameter that is only evaluated during creation of the node
        value = param_manager.declare_and_get_value(
            "a", 3.14, pmg.ParameterType.DOUBLE, "A double parameter"
        )

        if value.as_double() > 5:
            self.activate = True
            print(f"activated | Value is {value.as_double()}")
        else:
            self.activate = False
            print(f"not activated | Value is {value.as_double()}")


class ExampleNode(Node):

    def __init__(self):
        super().__init__("TestNode")

        self.param_manager = RCLPYValueManager(self)
        self.module = ROSIndependentSoftwareModule(self.param_manager)


if __name__ == "__main__":
    rclpy.init()  # Order does not matter here
    pmg.init([("a", 18.4)])
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()
