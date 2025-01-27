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
namespace tam::pmg
{
/// @brief A parameter manager that manages the values of the parameters internally.
class ParamValueManager : public MgmtInterface, public UserInterface
{
public:
  typedef std::shared_ptr<ParamValueManager> SharedPtr;
  typedef std::unique_ptr<ParamValueManager> UniquePtr;

private:
  std::unordered_set<std::string> param_names;
  std::unordered_map<std::string, std::string> param_descriptions;
  std::unordered_map<std::string, ParameterValue> param_values;
  std::size_t state_hash = std::rand();  // Init with random value

private:
  backend::InitializationBackend init_backend;

private:
  // Mark the methods inherited from the base class private, since they should only be used via a
  // base ptr
  bool has_parameter(const std::string & parameter_name) const override;
  std::unordered_set<std::string> list_parameters() const override;
  ParameterType get_type(const std::string & parameter_name) const override;
  std::string get_description(const std::string & parameter_name) const override;
  void set_value(const std::string & parameter_name, const param_value_variant_t & value) override;

public:
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
