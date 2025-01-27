// Copyright 2024 Simon Sagmeister
#include <iostream>
#include <param_management_cpp/base.hpp>
#include <param_management_cpp/param_value_manager.hpp>
#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
class MySoftwareModule
{
private:
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  double val_;
  void update_param_value()
  {
    val_ = param_manager_
             ->declare_and_get_value("a", 3.14, tam::pmg::ParameterType::DOUBLE, "Testparameter")
             .as_double();
  }

public:
  MySoftwareModule()
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
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager_; }
};
class ExampleNode : public rclcpp::Node
{
private:
  MySoftwareModule mod_;

public:
  explicit ExampleNode(MySoftwareModule const && module)
  : rclcpp::Node("TestNode"), mod_{std::move(module)}
  {
    // Connect your param manager to the callback
    // Important: don't forget to store your callback handle here, otherwise it will get deallocated
    auto callback_handle =
      tam::pmg::connect_param_manager_to_ros_cb(this, mod_.get_param_manager());
    // Declare all paramters from the param manager
    tam::pmg::declare_ros_params_from_param_manager(this, mod_.get_param_manager().get());

    // From now on, your module is supplied with the correct parameters from the overwrite file
    mod_.step();
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>(MySoftwareModule());
  rclcpp::spin(node);
  rclcpp::shutdown();
}
