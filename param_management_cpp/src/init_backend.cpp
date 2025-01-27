// Copyright 2024 Simon Sagmeister

#include "param_management_cpp/init_backend.hpp"
namespace tam::pmg::backend
{
void InitializationBackend::clear() { _storage.clear(); }
void InitializationBackend::set_init_val(std::string name, param_value_variant_t value)
{
  _storage[name] = value;
}
bool InitializationBackend::has_init_val(std::string name) { return _storage.count(name) > 0; }
param_value_variant_t InitializationBackend::get_init_val(std::string name)
{
  return _storage.at(name);
}
}  // namespace tam::pmg::backend
