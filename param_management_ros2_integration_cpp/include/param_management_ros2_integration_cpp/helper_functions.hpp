// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
namespace tam::pmg
{
/// @brief Goes through all the parameters in param_manager and declares them as ROS Parameters
/// @param node_instance Handle to the Node Instance
/// @param param_manager Handle to the Param Manager
void declare_ros_params_from_param_manager(
  rclcpp::Node * node_instance, MgmtInterface * param_manager);
/// @brief Ensures the ParamManager is updated on ROS Parameter changes
/// @param node_instance Handle to the Node Instance
/// @param param_manager Handle to the Param Manager
/// @return OnSetParametersCallbackHandle
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr connect_param_manager_to_ros_cb(
  rclcpp::Node * node_instance, MgmtInterface::SharedPtr param_manager);
/// @brief Loads Parameters from the provided YAML File and overwrites existing ROS Parameters. No
/// new parameters are declared
/// @param node_instance Handle to the Node Instance
/// @param path Path to .yml file
/// @param namesp Namespace inside yaml file in which the "ros__parameters" are declared
/// @param throw_if_overwr_non_existent_param woll throw an exception if trying to set a overwrite a
/// parameter that does not exist (default=true)
void load_overwrites_from_yaml(
  rclcpp::Node * node_instance, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param = true);
/// @brief Loads Parameters from the provided YAML File and overwrites existing Parameters inside
/// Param Manager. No new parameters are declared. Meant for Debugging the Algorithm Class without a
/// ROS-Node and still being able to test different param files
/// @param param_manager Handle to the Param Manager
/// @param path Path to .yml file
/// @param namesp Namespace inside yaml file in which the "ros__parameters" are declared
/// @param throw_if_overwr_non_existent_param woll throw an exception if trying to set a overwrite a
/// parameter that does not exist (default=false)
void load_overwrites_from_yaml(
  MgmtInterface * param_manager, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param = false, bool mandatory_overwrite = false);
/// @brief Load parameters from a yaml file for initialization.
/// @brief This means, if a parameter manager declares a parameter with a name, that is present in
/// the params file, the parameter will be initialized with the value from the override instead of
/// the default value given in the code.
/// @param path Path to .yml file
/// @param namesp Namespace inside yaml file in which the "ros__parameters" are declared
void init_from_yaml(const std::string & path, const std::string & namesp);
/// @brief Will throw if the parameter file provided with "--params-file" does not exist, contains
/// parameters that don't exist or does not contain the desired node. Call this function after node
/// is initialzed and before it is spinned.
/// @param argc argc argument main-function
/// @param argv argv argument of main-function
/// @param node pointer to node
void validate_param_overrides(int argc, char ** argv, rclcpp::Node * node);
}  // namespace tam::pmg
