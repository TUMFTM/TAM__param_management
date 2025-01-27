// Copyright 2024 Simon Sagmeister

#include "param_management_cpp/initialization.hpp"
namespace tam::pmg
{
void init(std::vector<std::pair<std::string, param_value_variant_t>> init_values)
{
  backend::InitializationBackend backend;
  for (auto const & entry : init_values) {
    backend.set_init_val(entry.first, entry.second);
  }
}
void reset()
{
  backend::InitializationBackend backend;
  backend.clear();
}
}  // namespace tam::pmg
