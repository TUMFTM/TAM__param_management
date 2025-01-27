// Copyright 2024 Simon Sagmeister
#include "param_management_cpp/param_value_manager.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
namespace tam::pmg
{
// Function definitions
bool ParamValueManager::has_parameter(const std::string & parameter_name) const
{
  return (param_names.count(parameter_name) > 0);
}
std::unordered_set<std::string> ParamValueManager::list_parameters() const { return param_names; }
ParameterType ParamValueManager::get_type(const std::string & parameter_name) const
{
  try {
    return param_values.at(parameter_name).get_type();
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
std::string ParamValueManager::get_description(const std::string & parameter_name) const
{
  try {
    return param_descriptions.at(parameter_name);
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
void ParamValueManager::set_value(
  const std::string & parameter_name, const param_value_variant_t & value)
{
  try {
    param_values.at(parameter_name).set_value(value);
    state_hash++;
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  } catch (const exceptions::BadParameterSet & e) {
    throw exceptions::BadParameterType(parameter_name);
  }
}
ParameterValue ParamValueManager::get_value(const std::string & parameter_name) const
{
  try {
    return param_values.at(parameter_name);
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
void ParamValueManager::declare_parameter(
  const std::string & parameter_name, const param_value_variant_t & default_value,
  const ParameterType & param_type, const std::string & description)
{
  if (has_parameter(parameter_name)) {
    throw exceptions::RedeclarationOfExistingParameter(parameter_name);
  }
  param_names.insert(parameter_name);
  param_descriptions.insert({parameter_name, description});
  auto init_value = init_backend.has_init_val(parameter_name)
                      ? init_backend.get_init_val(parameter_name)
                      : default_value;
  try {
    param_values.insert({parameter_name, ParameterValue(param_type, init_value)});
  } catch (const exceptions::BadParameterSet & e) {
    throw exceptions::BadParameterType(parameter_name);
  }
  state_hash++;
}
ParameterValue ParamValueManager::declare_and_get_value(
  const std::string & parameter_name, const param_value_variant_t & default_value,
  const ParameterType & param_type, const std::string & description)
{
  if (!has_parameter(parameter_name)) {
    declare_parameter(parameter_name, default_value, param_type, description);
  }
  return get_value(parameter_name);
};
std::size_t ParamValueManager::get_state_hash() const { return state_hash; }
}  // namespace tam::pmg
