# Copyright 2024 Simon Sagmeister
import rclpy
from rclpy.node import Node
import rcl_interfaces
import param_management_py as pmg


def convert_pmg_param_value_to_python_type(parameter_value: pmg.ParameterValue):
    type_dict = {
        pmg.ParameterType.BOOL: lambda x: x.as_bool(),
        pmg.ParameterType.INTEGER: lambda x: x.as_int(),
        pmg.ParameterType.DOUBLE: lambda x: x.as_double(),
        pmg.ParameterType.STRING: lambda x: x.as_string(),
        pmg.ParameterType.BOOL_ARRAY: lambda x: x.as_bool_array(),
        pmg.ParameterType.INTEGER_ARRAY: lambda x: x.as_int_array(),
        pmg.ParameterType.DOUBLE_ARRAY: lambda x: x.as_double_array(),
        pmg.ParameterType.STRING_ARRAY: lambda x: x.as_string_array(),
        pmg.ParameterType.BYTE_ARRAY: lambda x: x.as_byte_array(),
    }
    return type_dict[parameter_value.get_type()](parameter_value)


def declare_ros_params_from_param_manager(
    node_instance: Node, param_manager: pmg.MgmtInterface
):
    """Function for declaring all paremeters in the paraemter manager as ros nodes

    Args:
        node_instance (rclpy.Node): the node
        param_manager (pmg.MgmtInterface): A param manager instance
    """
    for parameter_name in param_manager.list_parameters():
        param_val = param_manager.get_value(parameter_name)
        node_instance.declare_parameter(
            parameter_name,
            convert_pmg_param_value_to_python_type(param_val),
            descriptor=rcl_interfaces.msg.ParameterDescriptor(
                name=parameter_name,
                type=param_manager.get_type(parameter_name).value,
                description=param_manager.get_description(parameter_name),
            ),
        )


def connect_param_manager_to_ros_cb(
    node_instance: Node, param_manager: pmg.MgmtInterface, strict=True
):
    """_summary_

    Args:
        node_instance (rclpy.Node): _description_
        param_manager (pmg.MgmtInterface): _description_
        strict (bool, optional): Return . Defaults to True. If set to False then the callback
            indicates success even though there were parameter not declared on the param
            manager.
    """

    def _bound_on_param_set_callback(param_list: list[rclpy.Parameter]):
        # Create the response object
        param_result = rcl_interfaces.msg.SetParametersResult()

        # Check if the model has all required parameters
        param_available = [
            param_manager.has_parameter(param.name) for param in param_list
        ]

        # Check if all parameters can be set successfully

        if all(param_available):
            for param in param_list:
                param_manager.set_value(param.name, param.value)
            reason = (
                "Successfully set the following parameter values:\n"
                + "------------------------------------------------\n"
                + "\n".join(
                    [f"{param.name}: {str(param.value)}" for param in param_list]
                )
            )
            param_result.successful = True
        else:
            if strict:
                err_msgs = [
                    f"The wrapped model does not have the parameter: {param.name}"
                    for param, available_flag in zip(param_list, param_available)
                    if not available_flag
                ]
                reason = (
                    "The parameter change was rejected because:\n"
                    + "------------------------------------------------\n"
                    + "\n".join(err_msgs)
                )
            else:
                log_msgs = []

                for param, available_flag in zip(param_list, param_available):
                    if available_flag:
                        param_manager.set_value(param.name, param.value)
                        log_msgs.append(
                            f"Set parameter: {param.name}: {str(param.value)}"
                        )
                    else:
                        log_msgs.append(
                            f"The wrapped model does not have the parameter: {param.name}"
                        )

                reason = (
                    "OPERATING IN NON STRICT MODE. Trying to set parameters:\n"
                    + "-----------------------------------------------------------\n"
                    + "\n".join(err_msgs)
                )

        param_result.reason = reason

        return param_result

    node_instance.add_on_set_parameters_callback(_bound_on_param_set_callback)
