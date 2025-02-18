// Copyright 2024 Simon Sagmeister

#include "param_management_ros2_integration_cpp/helper_functions.hpp"

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "param_management_cpp/initialization.hpp"
#include "param_management_ros2_integration_cpp/utils.hpp"
namespace tam::pmg
{
void load_overwrites_from_yaml(
  rclcpp::Node * node_instance, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param)
{
  rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  std::vector<rclcpp::Parameter> params = map[namesp];
  for (const auto & param : params) {
    if (!node_instance->has_parameter(param.get_name()) && throw_if_overwr_non_existent_param) {
      throw rclcpp::exceptions::ParameterNotDeclaredException(
        "Trying to overwrite non-existent Parameter " + param.get_name());
    }
    node_instance->set_parameter(param);
  }
}
void load_overwrites_from_yaml(
  MgmtInterface * param_manager, const std::string & path, const std::string & namesp,
  bool throw_if_overwr_non_existent_param, bool mandatory_overwrite)
{
  rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  std::vector<rclcpp::Parameter> params = map[namesp];
  if (mandatory_overwrite) {
    // Go through param_manager and throw if param not in map
    for (const auto & param : param_manager->list_parameters()) {
      if (!std::any_of(params.begin(), params.end(), [param](const rclcpp::Parameter & i) {
            return i.get_name() == param;
          })) {
        throw rclcpp::exceptions::ParameterNotDeclaredException(
          "The following parameter is not overwritten in yaml file (" + path + "): " + param +
          " (This is mandatory!)");
      }
    }
  }

  for (const auto & param : params) {
    if (!param_manager->has_parameter(param.get_name())) {
      if (throw_if_overwr_non_existent_param) {
        throw rclcpp::exceptions::ParameterNotDeclaredException(
          "Trying to overwrite non-existent Parameter " + param.get_name());
      } else {
        continue;
      }
    }
    utils::set_from_ros_param(param_manager, param);
  }
}
void init_from_yaml(const std::string & path, const std::string & namesp)
{
  rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  std::vector<rclcpp::Parameter> params = map[namesp];
  std::vector<std::pair<std::string, param_value_variant_t>> init_values;

  for (const auto & param : params) {
    init_values.push_back(
      {param.get_name(), utils::convert_to_pmg_parameter_variant_t(param.get_parameter_value())});
  }
  init(init_values);
}
void declare_ros_params_from_param_manager(
  rclcpp::Node * node_instance, MgmtInterface * param_manager)
{
  for (std::string const & name : param_manager->list_parameters()) {
    utils::declare_parameter_on_node(
      node_instance, name, param_manager->get_value(name), param_manager->get_description(name));
  }
}
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr connect_param_manager_to_ros_cb(
  rclcpp::Node * node_instance, MgmtInterface::SharedPtr param_manager)
{
  // Use a lambda here to capture the SharedPtr and convert it to a raw_ptr before calling the
  // non-owning param_set_callback function
  return node_instance->add_on_set_parameters_callback([param_manager](auto params) {
    return utils::param_set_callback(params, param_manager.get());
  });
}
void validate_param_overrides(int argc, char ** argv, rclcpp::Node * node)
{
  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::string node_name = node->get_fully_qualified_name();
  // Get param_file path from argv
  std::vector<std::string> param_file_names{};
  for (std::size_t i = 0; i < arguments.size(); i++) {
    if (arguments.at(i) == "--params-file") {
      param_file_names.emplace_back(arguments.at(i + 1));
    }
  }
  if (param_file_names.empty()) {
    return;
  }

  std::vector<std::string> param_file_abs_names{};

  // Check if absolute path and convert if relative path
  for (auto & param_file : param_file_names) {
    if (param_file.rfind("/", 0) != 0) {
      std::string new_param_file =
        std::filesystem::current_path().string() + std::string("/") + param_file;
      param_file_abs_names.push_back(new_param_file);
    } else {
      param_file_abs_names.push_back(param_file);
    }
  }
  // check if Param file exists and throw
  for (auto & param_file : param_file_abs_names) {
    if (!std::filesystem::exists(param_file)) {
      throw std::invalid_argument(
        node_name + ": You were trying to set a non-existing parameter file: " + param_file);
    }
  }

  // Build param map from all parameter files
  rclcpp::ParameterMap param_map{};
  for (auto & param_file : param_file_abs_names) {
    auto param_map_tmp = rclcpp::parameter_map_from_yaml_file(param_file);

    for (auto param : param_map_tmp) {
      param_map.insert(param);
    }
  }

  if (!param_map.count(node_name)) {  // Key not in map
    throw std::invalid_argument(
      node_name + ": Key (fully qualified node name) not part of param-file: " + node_name);
  }

  auto parameters = param_map[node_name];

  // Verify parameters
  for (const auto & param : parameters) {
    if (!node->has_parameter(param.get_name())) {
      throw std::invalid_argument(
        node_name + ": You were trying to set a non-existing parameter: " + param.get_name());
    }
  }
}
}  // namespace tam::pmg
