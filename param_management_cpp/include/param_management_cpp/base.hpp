// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <variant>
#include <vector>

#include "param_management_cpp/types.hpp"
namespace tam::pmg
{
/// @brief Management interface for a parameter manager.
/// @brief This is mainly used for example for:
/// @brief - Connecting the param manager to a node
/// @brief - Load parameters into a manager from a file
/// @brief - Check if parameters from a file were set correctly.
/// @brief - Composing multiple parameter managers
class MgmtInterface
{
public:
  typedef std::shared_ptr<MgmtInterface> SharedPtr;
  typedef std::unique_ptr<MgmtInterface> UniquePtr;
  virtual ~MgmtInterface() = default;
  // Interface methods
public:
  virtual bool has_parameter(const std::string & parameter_name) const = 0;
  virtual std::unordered_set<std::string> list_parameters() const = 0;
  virtual ParameterType get_type(const std::string & parameter_name) const = 0;
  virtual std::string get_description(const std::string & parameter_name) const = 0;

  /// @brief Set parameter value
  virtual void set_value(
    const std::string & parameter_name, const param_value_variant_t & value) = 0;

  /// @brief Get parameter value
  virtual ParameterValue get_value(const std::string & parameter_name) const = 0;
};
/// @brief The interface for the user of the library to declare and access parameters
class UserInterface
{
public:
  typedef std::shared_ptr<UserInterface> SharedPtr;
  typedef std::unique_ptr<UserInterface> UniquePtr;
  virtual ~UserInterface() = default;

public:
  virtual ParameterValue get_value(const std::string & parameter_name) const = 0;

  /// @brief Declare a parameter on the param manager.
  /// @throw RedeclarationOfExistingParameter: If the parameter is already defined
  virtual void declare_parameter(
    const std::string & parameter_name, const param_value_variant_t & default_value,
    const ParameterType & param_type, const std::string & description) = 0;

  /// @brief The function does the following
  /// @brief   1. Declare the parameter on the param manager if it is not already declared.
  /// @brief   2. Return the current parameter value. (This will be the defaul value if the
  /// parameter was not already declared before calling this function)
  virtual ParameterValue declare_and_get_value(
    const std::string & parameter_name, const param_value_variant_t & default_value,
    const ParameterType & param_type, const std::string & description) = 0;
  /// @brief Get a state hash, that reflect the interal state of the param manager.
  /// @brief This updates every time a parameter is declared or a parameter value is updated.
  /// @return hash
  virtual std::size_t get_state_hash() const = 0;
};
}  // namespace tam::pmg
