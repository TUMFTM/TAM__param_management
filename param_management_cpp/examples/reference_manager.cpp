// Copyright 2024 Simon Sagmeister
#include <iostream>

#include "param_management_cpp/param_reference_manager.hpp"
struct ModuleParameters
{
  double a, b, c;
};
class SoftwareModule
{
private:
  ModuleParameters params;  // Use a struct for caching and simpler access due to autocompletion
  tam::pmg::ParamReferenceManager::SharedPtr param_manager =
    std::make_shared<tam::pmg::ParamReferenceManager>();
  void declare_parameters()
  {
    // Declare the parameters.
    // The reference manager automatically keeps the values inside the struct up to date.
    // Do not write manually to the param struct!
    // clang-format off
    param_manager->declare_parameter("a", &params.a ,1.0, tam::pmg::ParameterType::DOUBLE, "Description of a"); // NOLINT
    param_manager->declare_parameter("b", &params.b ,2.0, tam::pmg::ParameterType::DOUBLE, "Description of b"); // NOLINT
    param_manager->declare_parameter("c", &params.c ,3.0, tam::pmg::ParameterType::DOUBLE, "Description of c"); // NOLINT
    // clang-format on
  }

public:
  SoftwareModule()
  {
    // Declare the parameters here and initialize the values in the struct
    declare_parameters();
  }
  double step() const { return params.a * params.b * params.c; }
};
int main()
{
  auto module = SoftwareModule();
  std::cout << "Result: " << module.step() << std::endl;
  return 0;
}
