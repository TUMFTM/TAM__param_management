// Copyright 2024 Simon Sagmeister
#include "param_management_cpp/param_manager_composer.hpp"
namespace tam::pmg
{
// Constructor
ParamManagerComposer::ParamManagerComposer(std::vector<std::shared_ptr<MgmtInterface>> list)
: param_manager_list_(list)
{
  for (const auto & parameter_name : list_parameters()) {
    ensure_param_consistency(parameter_name);
  }
}
void ParamManagerComposer::ensure_param_consistency(const std::string & parameter_name) const
{
  std::vector<ParameterValue> values;
  // Get all the different values from all param managers
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      values.push_back(manager->get_value(parameter_name));
    }
  }
  // Compare every element to the first element
  for (const auto & element : values) {
    ParameterValueComparisonResult equality = values[0] == element;
    if (!equality.type_equal)
      throw exceptions::ParameterCompositionConflictingTypes(parameter_name);
    if (!equality.value_equal)
      throw exceptions::ParameterCompositionConflictingValues(parameter_name);
  }
}
bool ParamManagerComposer::has_parameter(const std::string & parameter_name) const
{
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      return true;
    }
  }
  return false;
}
std::unordered_set<std::string> ParamManagerComposer::list_parameters() const
{
  std::unordered_set<std::string> out;
  for (const auto & manager : param_manager_list_) {
    out.insert(manager->list_parameters().begin(), manager->list_parameters().end());
  }
  return out;
}
ParameterType ParamManagerComposer::get_type(const std::string & parameter_name) const
{
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      return manager->get_type(parameter_name);
    }
  }
  throw exceptions::NonDeclaredParameterAccess(parameter_name);
}
std::string ParamManagerComposer::get_description(const std::string & parameter_name) const
{
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      return manager->get_description(parameter_name);
    }
  }
  throw exceptions::NonDeclaredParameterAccess(parameter_name);
}
void ParamManagerComposer::set_value(
  const std::string & parameter_name, const param_value_variant_t & value)
{
  bool set_at_least_one = false;
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      set_at_least_one = true;
      manager->set_value(parameter_name, value);
    }
  }
  if (!set_at_least_one) throw exceptions::NonDeclaredParameterAccess(parameter_name);
}
ParameterValue ParamManagerComposer::get_value(const std::string & parameter_name) const
{
  ensure_param_consistency(parameter_name);
  for (const auto & manager : param_manager_list_) {
    if (manager->has_parameter(parameter_name)) {
      return manager->get_value(parameter_name);
    }
  }
  throw exceptions::NonDeclaredParameterAccess(parameter_name);
}
}  // namespace tam::pmg
