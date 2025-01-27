// Copyright 2024 Simon Sagmeister
#include "param_management_ros2_integration_cpp/utils.hpp"
namespace tam::pmg::utils
{
void declare_parameter_on_node(
  rclcpp::Node * node_instance, const std::string & parameter_name,
  const param_value_variant_t & default_value, const ParameterType & param_type,
  const std::string & description)
{
  try {
    declare_parameter_on_node(
      node_instance, parameter_name, ParameterValue(param_type, default_value), description);
  } catch (const exceptions::BadParameterSet & e) {
    throw exceptions::BadParameterType(parameter_name);
  }
}
void declare_parameter_on_node(
  rclcpp::Node * node_instance, const std::string & parameter_name, ParameterValue default_value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  node_instance->declare_parameter(
    parameter_name, utils::convert_to_rclcpp_parameter_value(default_value), descriptor);
}
bool set_from_ros_param(MgmtInterface * param_manager, const rclcpp::Parameter & param)
{
  param_manager->set_value(
    param.get_name(), utils::convert_to_pmg_parameter_variant_t(param.get_parameter_value()));
  return true;
}
rcl_interfaces::msg::SetParametersResult param_set_callback(
  const std::vector<rclcpp::Parameter> & parameters, MgmtInterface * param_manager)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & param : parameters) {
    if ((param_manager->has_parameter(param.get_name()) &&
         !set_from_ros_param(param_manager, param))) {
      result.successful = false;
      result.reason = "Could not set at least one parameter";
      return result;
    }
  }
  result.successful = true;
  result.reason = "Successfully set parameter values";
  return result;
}
rclcpp::ParameterValue convert_to_rclcpp_parameter_value(ParameterValue const & parameter_value)
{
// clang-format off
  #define TO_RCLCPP_PARAM_VALUE(enum_val, typename, name) \
    case ParameterType::enum_val: return rclcpp::ParameterValue(parameter_value.as_##name()); // NOLINT
  // clang-format on

  switch (parameter_value.get_type()) {
    // clang-format off
    TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(TO_RCLCPP_PARAM_VALUE)
    // clang-format on
    default:
      throw exceptions::UnsupportedParameterType(static_cast<uint8_t>(parameter_value.get_type()));
      break;
  }
}
param_value_variant_t convert_to_pmg_parameter_variant_t(
  rclcpp::ParameterValue const & parameter_value)
{
// clang-format off
  #define RETURN_CPP_TYPE(enum_val, typename, name)                 \
    case rclcpp::ParameterType::PARAMETER_##enum_val: return parameter_value.get<rclcpp::ParameterType::PARAMETER_##enum_val>(); // NOLINT
  // clang-format on

  rclcpp::ParameterType rclcpp_param_type = parameter_value.get_type();
  switch (rclcpp_param_type) {
    // clang-format off
    TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(RETURN_CPP_TYPE)
    default:
      // clang-format on
      throw exceptions::UnsupportedParameterType(static_cast<uint8_t>(rclcpp_param_type));
      break;
  }
    // clang-format off
  #undef RETURN_CPP_TYPE
  // clang-format on
}
ParameterValue convert_to_pmg_parameter_value(rclcpp::ParameterValue const & parameter_value)
{
  rclcpp::ParameterType rclcpp_param_type = parameter_value.get_type();
  // Should be safe here since rclcpp should already ensure, that the parameter is of the correct
  // type
  return ParameterValue(
    static_cast<ParameterType>(rclcpp_param_type),
    convert_to_pmg_parameter_variant_t(parameter_value));
}
}  // namespace tam::pmg::utils
