// Copyright 2024 Simon Sagmeister
#include <iostream>
#include <param_management_cpp/base.hpp>
#include <param_management_cpp/param_value_manager.hpp>
#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <param_management_ros2_integration_cpp/rclcpp_value_manager.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
class MySoftwareModule
{
private:
  tam::pmg::UserInterface::SharedPtr param_manager_{};
  double val_;
  void update_param_value()
  {
    val_ = param_manager_
             ->declare_and_get_value("a", 3.14, tam::pmg::ParameterType::DOUBLE, "Testparameter")
             .as_double();
  }

public:
  explicit MySoftwareModule(tam::pmg::UserInterface::SharedPtr param_manager)
  : param_manager_{param_manager}
  {
    // CAREFUL: Independent from the overwrite, you will always get the default value here
    // since you create your module before connecting with a node or loading parameters
    update_param_value();
    std::cout << "Value (Constructor): " << val_ << std::endl;
  }
  void step()
  {
    update_param_value();
    std::cout << "Value (Step): " << val_ << std::endl;
  }
};
class ExampleNode : public rclcpp::Node
{
private:
  tam::pmg::RCLCPPValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::RCLCPPValueManager>(this);
  MySoftwareModule mod_{param_manager_};

public:
  ExampleNode() : rclcpp::Node("TestNode")
  {
    // You don't need the helper functions here, since you use the RCLCPP Param Manager
    mod_.step();
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
