// Copyright 2024 Simon Sagmeister
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "param_management_cpp/init_backend.hpp"
#include "rclcpp/node.hpp"
namespace tam::pmg
{
/// @brief A parameter manager that uses the node itself to handle parameters
class RCLCPPValueManager : public UserInterface
{
public:
  typedef std::shared_ptr<RCLCPPValueManager> SharedPtr;
  typedef std::unique_ptr<RCLCPPValueManager> UniquePtr;

private:
  rclcpp::Node * node_ptr_{};
  std::size_t state_hash_ = std::numeric_limits<std::size_t>::max();  // Init with max value
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    state_hash_update_callback_handle_;

private:
  backend::InitializationBackend init_backend_;

public:
  explicit RCLCPPValueManager(rclcpp::Node * node_ptr) : node_ptr_{node_ptr}
  {
    // Change the state hash every time a param action is called.
    state_hash_update_callback_handle_ = node_ptr_->add_on_set_parameters_callback([this](auto) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      state_hash_++;
      return result;
    });
  }
  ParameterValue get_value(const std::string & parameter_name) const override;

  /// @brief Declare a parameter on the param manager.
  /// @throw RedeclarationOfExistingParameter: If the parameter is already defined
  void declare_parameter(
    const std::string & parameter_name, const param_value_variant_t & default_value,
    const ParameterType & param_type, const std::string & description) override;

  /// @brief The function does the following
  /// @brief   1. Declare the parameter on the param manager if it is not already declared.
  /// @brief   2. Return the current parameter value. (This will be the defaul value if the
  /// parameter was not already declared before calling this function)
  ParameterValue declare_and_get_value(
    const std::string & parameter_name, const param_value_variant_t & default_value,
    const ParameterType & param_type, const std::string & description) override;
  /// @brief Get a state hash, that reflect the interal state of the param manager.
  /// @brief This updates every time a parameter is declared or a parameter value is updated.
  /// @return hash
  std::size_t get_state_hash() const override;
};
}  // namespace tam::pmg
