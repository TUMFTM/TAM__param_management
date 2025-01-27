// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "param_management_cpp/base.hpp"
namespace tam::pmg::utils
{
/// @brief Convert a `tam::pmg::ParameterValue` to a `rclcpp::ParameterValue` type
/// @param parameter_value
/// @return
rclcpp::ParameterValue convert_to_rclcpp_parameter_value(ParameterValue const & parameter_value);
/// @brief Overload for creating a tam::pmg::ParameterValue from a rclcpp param value
/// @param parameter_value
/// @return
ParameterValue convert_to_pmg_parameter_value(rclcpp::ParameterValue const & parameter_value);
/// @brief Overload for creating a tam::pmg::param_value_variant_t from a rclcpp param value
/// @param parameter_value
/// @return
param_value_variant_t convert_to_pmg_parameter_variant_t(
  rclcpp::ParameterValue const & parameter_value);
/// @brief Declare a parameter on a rclcpp::Node
/// @param node_instance
/// @param parameter_name
/// @param default_value
/// @param param_type
/// @param description
void declare_parameter_on_node(
  rclcpp::Node * node_instance, const std::string & parameter_name,
  const param_value_variant_t & default_value, const ParameterType & param_type,
  const std::string & description);
void declare_parameter_on_node(
  rclcpp::Node * node_instance, const std::string & parameter_name, ParameterValue default_value,
  const std::string & description);
/// @brief Update Parameter in Param Manager with a Ros Parameter
/// @param param_manager Parameter Manager is updated with the provided Ros Parameter
/// @param param Input Ros Parameter
/// @return Boolean if successfull
bool set_from_ros_param(MgmtInterface * param_manager, const rclcpp::Parameter & param);
/// @brief Callback function that is called in case a Paramter is changed on runtime
/// @param parameters Parameters that are changing
/// @param param_manager Handle to the Param Manager
/// @return SetParametersResult
rcl_interfaces::msg::SetParametersResult param_set_callback(
  const std::vector<rclcpp::Parameter> & parameters, MgmtInterface * param_manager);
}  // namespace tam::pmg::utils
