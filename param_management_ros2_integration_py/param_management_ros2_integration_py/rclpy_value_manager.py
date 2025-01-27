# Copyright 2024 Simon Sagmeister
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
import param_management_py as pmg
from typing import Any
import numpy as np


from param_management_py.init_backend import InitializationBackend
from param_management_ros2_integration_py.helper_functions import (
    convert_pmg_param_value_to_python_type,
)
from param_management_py.exceptions import BadParameterSet, BadParameterType


# Implements the following base class pmg.UserInterface
class RCLPYValueManager:
    def __init__(self, node: Node):
        self.node = node
        self.init_backend = InitializationBackend()
        generator = np.random.default_rng()
        self.state_hash = generator.integers(
            0, np.iinfo(np.uint64).max, dtype=np.uint64
        )

        # Change the state hash every time a param action is called.
        self.state_hash_update_callback_handle = (
            self.node.add_on_set_parameters_callback(self._update_state_hash)
        )

    def _update_state_hash(self, _):
        result = SetParametersResult()
        result.successful = True
        self.state_hash += 1
        return result

    def get_value(self, parameter_name: str) -> pmg.ParameterValue:
        # Implementation needed based on how parameters are stored/retrieved
        param = self.node.get_parameter(parameter_name)
        # rclpy should already garantuee the type correctness.
        return pmg.ParameterValue(pmg.ParameterType(param.type_.value), param.value)

    def declare_parameter(
        self,
        parameter_name: str,
        default_value: Any,
        param_type: pmg.ParameterType,
        description: str,
    ) -> None:
        descriptor = ParameterDescriptor()
        descriptor.description = description
        # If rclpy has an init value in its background, it overwrites the init value of the param manager here
        init_val = (
            self.init_backend.get_init_val(parameter_name)
            if self.init_backend.has_init_val(parameter_name)
            else default_value
        )

        try:
            self.node.declare_parameter(
                parameter_name,
                # Use cpp's type system to garantue the type
                convert_pmg_param_value_to_python_type(
                    pmg.ParameterValue(param_type, init_val)
                ),
                descriptor,
            )
        except BadParameterSet:
            raise BadParameterType(parameter_name)

    def declare_and_get_value(
        self, parameter_name: str, default_value: Any, param_type: Any, description: str
    ) -> pmg.ParameterValue:
        # Implementation needed based on the combined logic of declaring and getting the parameter value
        if not self.node.has_parameter(parameter_name):
            self.declare_parameter(
                parameter_name, default_value, param_type, description
            )
        return self.get_value(parameter_name)

    def get_state_hash(self) -> int:
        return self.state_hash
