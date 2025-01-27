// Copyright 2024 Simon Sagmeister
#include <iostream>

#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
// A submodule with individual parameters
class SoftwareSubModuleA
{
private:
  tam::pmg::ParamValueManager::SharedPtr param_manager =
    std::make_shared<tam::pmg::ParamValueManager>();
  void declare_parameters() const
  {
    // Declare the parameters
    param_manager->declare_parameter("a", 1.0, tam::pmg::ParameterType::DOUBLE, "Description of a");
    param_manager->declare_parameter("b", 2.0, tam::pmg::ParameterType::DOUBLE, "Description of b");
    param_manager->declare_parameter(
      "c", 3.0, tam::pmg::ParameterType::DOUBLE,
      "Description of c");  // This parameter is shared among multiple parameter managers that are
                            // later composed. Bc of this they have to be declared with the same
                            // default value in order to be handled like a single parameter later
                            // on.
  }

public:
  SoftwareSubModuleA() { declare_parameters(); }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager; }
};
// Another submodule with individual parameters
class SoftwareSubModuleB
{
private:
  tam::pmg::ParamValueManager::SharedPtr param_manager =
    std::make_shared<tam::pmg::ParamValueManager>();
  void declare_parameters() const
  {
    // Declare the parameters
    param_manager->declare_parameter(
      "c", 3.0, tam::pmg::ParameterType::DOUBLE,
      "Description of c");  // This parameter is shared among multiple parameter managers that are
                            // later composed. Bc of this they have to be declared with the same
                            // default value in order to be handled like a single parameter later
                            // on.
    param_manager->declare_parameter("d", 2.0, tam::pmg::ParameterType::DOUBLE, "Description of d");
    param_manager->declare_parameter("e", 1.0, tam::pmg::ParameterType::DOUBLE, "Description of e");
  }

public:
  SoftwareSubModuleB() { declare_parameters(); }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager; }
};
// A module that consists of 2 individual submodules
class SoftwareModule
{
private:
  SoftwareSubModuleA a;
  SoftwareSubModuleB b;

  // Setup the composing parameter manager
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer =
    std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        a.get_param_manager(), b.get_param_manager()});

public:
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager_composer; }
};
int main()
{
  auto module = SoftwareModule();
  auto params = module.get_param_manager()->list_parameters();
  std::cout << "Available Parameters: ";
  for (auto const & param : params) {
    std::cout << param << ", ";
  }
  std::cout << std::endl;
  return 0;
}
