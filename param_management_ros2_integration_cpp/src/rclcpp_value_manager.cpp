// Copyright 2024 Simon Sagmeister

#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <param_management_ros2_integration_cpp/rclcpp_value_manager.hpp>
#include <param_management_ros2_integration_cpp/utils.hpp>
namespace tam::pmg
{
ParameterValue RCLCPPValueManager::get_value(const std::string & parameter_name) const
{
  return utils::convert_to_pmg_parameter_value(
    node_ptr_->get_parameter(parameter_name).get_parameter_value());
}
void RCLCPPValueManager::declare_parameter(
  const std::string & parameter_name, const param_value_variant_t & default_value,
  const ParameterType & param_type, const std::string & description)
{
  auto init_value = init_backend_.has_init_val(parameter_name)
                      ? init_backend_.get_init_val(parameter_name)
                      : default_value;
  // If rclcpp has already an init value, this overwrites the default value of the tam pmg pkg
  utils::declare_parameter_on_node(node_ptr_, parameter_name, init_value, param_type, description);
}
ParameterValue RCLCPPValueManager::declare_and_get_value(
  const std::string & parameter_name, const param_value_variant_t & default_value,
  const ParameterType & param_type, const std::string & description)
{
  if (!node_ptr_->has_parameter(parameter_name)) {
    declare_parameter(parameter_name, default_value, param_type, description);
  }
  return get_value(parameter_name);
}
std::size_t RCLCPPValueManager::get_state_hash() const { return state_hash_; }
}  // namespace tam::pmg
