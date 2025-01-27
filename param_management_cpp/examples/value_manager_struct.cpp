// Copyright 2024 Simon Sagmeister
#include <iostream>

#include "param_management_cpp/param_value_manager.hpp"
struct ModuleParameters
{
  double a, b, c;
};
class SoftwareModule
{
private:
  ModuleParameters params;  // Use a struct for caching and simpler access due to autocompletion
  // Remember the parameter state loaded into the struct -> only update on change
  std::size_t previous_param_state_hash = 0;
  tam::pmg::ParamValueManager::SharedPtr param_manager =
    std::make_shared<tam::pmg::ParamValueManager>();
  void declare_and_update_parameters()
  {
    // clang-format off
    // Declare the parameters if they do not already exist and return their current values
    params.a = param_manager->declare_and_get_value("a", 1.0, tam::pmg::ParameterType::DOUBLE, "Description of a").as_double(); // NOLINT
    params.b = param_manager->declare_and_get_value("b", 2.0, tam::pmg::ParameterType::DOUBLE, "Description of b").as_double(); // NOLINT
    params.c = param_manager->declare_and_get_value("c", 3.0, tam::pmg::ParameterType::DOUBLE, "Description of c").as_double(); // NOLINT
    previous_param_state_hash = param_manager->get_state_hash();
    // clang-format on
  }

public:
  SoftwareModule()
  {
    // Declare the parameters here and initialize the values in the struct
    declare_and_update_parameters();
  }
  double step()
  {
    // Update the parameter values inside the struct in case parameters inside the param manager
    // were changed
    if (param_manager->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
    }
    return params.a * params.b * params.c;
  }
};
int main()
{
  auto module = SoftwareModule();
  std::cout << "Result: " << module.step() << std::endl;
  return 0;
}
