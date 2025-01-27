// Copyright 2024 Simon Sagmeister
#include "param_management_cpp/param_reference_manager.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
namespace tam::pmg
{
// Function definitions
bool ParamReferenceManager::has_parameter(const std::string & parameter_name) const
{
  return (param_names.count(parameter_name) > 0);
}
std::unordered_set<std::string> ParamReferenceManager::list_parameters() const
{
  return param_names;
}
ParameterType ParamReferenceManager::get_type(const std::string & parameter_name) const
{
  try {
    return param_references.at(parameter_name).get_type();
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
std::string ParamReferenceManager::get_description(const std::string & parameter_name) const
{
  try {
    return param_descriptions.at(parameter_name);
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
void ParamReferenceManager::set_value(
  const std::string & parameter_name, const param_value_variant_t & value)
{
  try {
    param_references.at(parameter_name).set_value(value);
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  } catch (const exceptions::BadParameterSet & e) {
    throw exceptions::BadParameterType(parameter_name);
  }
}
ParameterValue ParamReferenceManager::get_value(const std::string & parameter_name) const
{
  try {
    return param_references.at(parameter_name).to_value();
  } catch (const std::out_of_range & oor) {
    throw exceptions::NonDeclaredParameterAccess(parameter_name);
  }
}
void ParamReferenceManager::declare_parameter(
  const std::string & parameter_name, param_ptr_variant_t storage_ptr,
  const param_value_variant_t & default_value, const ParameterType & param_type,
  const std::string & description)
{
  if (this->has_parameter(parameter_name)) {
    throw exceptions::RedeclarationOfExistingParameter(parameter_name);
  }
  param_names.insert(parameter_name);
  param_descriptions.insert({parameter_name, description});
  auto init_value = init_backend.has_init_val(parameter_name)
                      ? init_backend.get_init_val(parameter_name)
                      : default_value;
  try {
    param_references.insert(
      {parameter_name, ParameterReference(param_type, storage_ptr, init_value)});
  } catch (const exceptions::BadParameterSet & e) {
    throw exceptions::BadParameterType(parameter_name);
  }
}
}  // namespace tam::pmg
