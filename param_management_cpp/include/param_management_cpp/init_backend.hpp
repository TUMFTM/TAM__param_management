// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "param_management_cpp/types.hpp"
namespace tam::pmg::backend
{
class InitializationBackend
{
  inline static std::unordered_map<std::string, param_value_variant_t> _storage;

public:
  void clear();
  void set_init_val(std::string name, param_value_variant_t value);
  bool has_init_val(std::string name);
  param_value_variant_t get_init_val(std::string name);
};
}  // namespace tam::pmg::backend
