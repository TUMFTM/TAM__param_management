// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "param_management_cpp/init_backend.hpp"
namespace tam::pmg
{
/// @brief Param Manager that does not store the parameters internally but instead writes the value
/// directly to a memory adress.
/// @note DISCLAIMER
/// @note ======================================================================
/// @note Be very careful when using this implementation !!!!
/// @note ======================================================================
/// @note You have to ensure the following things when using this implementation:
/// @note    1. You are responsible for keeping the memory addresses, that will be stored
/// @note       inside the param manager (during declaration) valid. Otherwise the program will
/// @note       either segfault (bestcase) or undefined behavior will occur.
/// @note    2. Do not overwrite the values managed by this param manager yourself
/// @note       by just writing values to the memory address managed by the param manager.
/// @note       This could lead to a situation where the parameters get out of sync between
/// @note       multiple param managers inside a composer.
class ParamReferenceManager : public MgmtInterface
{
public:
  typedef std::shared_ptr<ParamReferenceManager> SharedPtr;
  typedef std::unique_ptr<ParamReferenceManager> UniquePtr;

private:
  std::unordered_set<std::string> param_names;
  std::unordered_map<std::string, std::string> param_descriptions;
  std::unordered_map<std::string, ParameterReference> param_references;

private:
  // Backend for initialization
  backend::InitializationBackend init_backend;

  // Mark the methods inherited from the base class private, since they should only be used via a
  // base ptr
private:
  bool has_parameter(const std::string & parameter_name) const override;
  std::unordered_set<std::string> list_parameters() const override;
  ParameterType get_type(const std::string & parameter_name) const override;
  std::string get_description(const std::string & parameter_name) const override;
  void set_value(const std::string & parameter_name, const param_value_variant_t & value) override;
  ParameterValue get_value(const std::string & parameter_name) const override;

public:
  /// @brief Declare a that is stored at the corresponding raw memory adress.
  void declare_parameter(
    const std::string & parameter_name, param_ptr_variant_t storage_ptr,
    const param_value_variant_t & default_value, const ParameterType & param_type,
    const std::string & description);
};
}  // namespace tam::pmg
