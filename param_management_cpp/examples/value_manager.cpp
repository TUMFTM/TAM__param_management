// Copyright 2024 Simon Sagmeister
#include <iostream>

#include "param_management_cpp/param_value_manager.hpp"
class SoftwareModule
{
private:
  tam::pmg::ParamValueManager::SharedPtr param_manager =
    std::make_shared<tam::pmg::ParamValueManager>();
  void declare_parameters() const
  {
    // Declare the parameters
    param_manager->declare_parameter("a", 1.0, tam::pmg::ParameterType::DOUBLE, "Description of a");
    param_manager->declare_parameter("b", 2.0, tam::pmg::ParameterType::DOUBLE, "Description of b");
    param_manager->declare_parameter("c", 3.0, tam::pmg::ParameterType::DOUBLE, "Description of c");
  }

public:
  SoftwareModule()
  {
    // Declare the parameters here and initialize the values in the struct
    declare_parameters();
  }
  double step() const
  {
    // Be careful, getting a parameter directly from the manager each time is
    // comparably inefficient. It is better to get the value once and store it in a local variable.
    // For this, see the example "value_manager_struct.cpp"
    return param_manager->get_value("a").as_double() * param_manager->get_value("b").as_double() *
           param_manager->get_value("c").as_double();
  }
};
int main()
{
  auto module = SoftwareModule();
  std::cout << "Result: " << module.step() << std::endl;
  return 0;
}
