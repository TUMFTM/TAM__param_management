// Copyright 2024 Simon Sagmeister
#pragma once
#include <string>
#include <utility>
#include <vector>

#include "param_management_cpp/init_backend.hpp"
#include "param_management_cpp/types.hpp"
namespace tam::pmg
{
/// @brief Set the initialization values for a flexible number of parameters.
/// @brief Can be called multiple times. Results from each call will be aggregated.
/// @note An initilization value, if present, will be used for instead of the default value given by
/// the declare_parameter method.
/// @param init_values: Vector of pairs [name, init_value]
void init(std::vector<std::pair<std::string, param_value_variant_t>> init_values);
/// @brief Reset the initialization layer from all previous call to init().
void reset();
}  // namespace tam::pmg
