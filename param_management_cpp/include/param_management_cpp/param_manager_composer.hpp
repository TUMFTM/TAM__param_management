// Copyright 2024 Simon Sagmeister
#pragma once
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_management_cpp/base.hpp"
namespace tam::pmg
{
/// @brief A helper class to access multiple parameter managers as a single instance.
/// @brief
/// @note Requires that parameters that are defined with the same name in multiple parameter
/// managers, have the same type and value
class ParamManagerComposer : public MgmtInterface
{
private:
  std::vector<std::shared_ptr<MgmtInterface>> param_manager_list_;
  void ensure_param_consistency(std::string const & param_name) const;

public:
  typedef std::shared_ptr<ParamManagerComposer> SharedPtr;
  typedef std::unique_ptr<ParamManagerComposer> UniquePtr;
  // Constructor
  explicit ParamManagerComposer(std::vector<std::shared_ptr<MgmtInterface>> param_manager_list);

  // Mark the methods inherited from the base class private, since they should only be used via a
  // base ptr
private:
  bool has_parameter(const std::string & parameter_name) const override;
  std::unordered_set<std::string> list_parameters() const override;
  ParameterType get_type(const std::string & parameter_name) const override;
  std::string get_description(const std::string & parameter_name) const override;
  void set_value(const std::string & parameter_name, const param_value_variant_t & value) override;
  ParameterValue get_value(const std::string & parameter_name) const override;
};
}  // namespace tam::pmg
