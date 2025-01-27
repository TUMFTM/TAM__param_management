// Copyright 2024 Simon Sagmeister
#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <variant>
#include <vector>
namespace tam::pmg
{
namespace exceptions
{
struct BadParameterType : public std::runtime_error
{
  explicit BadParameterType(std::string const & parameter_name)
  : std::runtime_error(std::string(
      "ParamMangement | Incompatible parameter type while handling the parameter: " +
      parameter_name))
  {
  }
};
struct BadParameterCast : public std::runtime_error
{
  BadParameterCast()
  : std::runtime_error(
      "ParamMangement | Tried casting a parameter to a different type than it was declared with! "
      "| This is an implementation issue.")
  {
  }
};
struct BadParameterSet : public std::runtime_error
{
  BadParameterSet()
  : std::runtime_error(
      "ParamMangement | Setting the parameter failed because the given type was different than the "
      "declared type!")
  {
  }
};
struct UnsupportedParameterType : public std::runtime_error
{
  explicit UnsupportedParameterType(int type_id_)
  : std::runtime_error(
      std::string("ParamMangement | Unsupport Type with type ID: ") + std::to_string(type_id_) +
      " | This is an implementation issue.")
  {
  }
};
struct NonDeclaredParameterAccess : public std::runtime_error
{
  explicit NonDeclaredParameterAccess(std::string const & parameter_name)
  : std::runtime_error(
      std::string("ParamMangement | Tried accessing non declared parameter: " + parameter_name) +
      " | This is an implementation issue.")
  {
  }
};
struct RedeclarationOfExistingParameter : public std::runtime_error
{
  explicit RedeclarationOfExistingParameter(std::string const & parameter_name)
  : std::runtime_error(
      std::string("ParamMangement | Tried declaring an already existing parameter: ") +
      parameter_name + " | This is an implementation issue.")
  {
  }
};
struct ParameterCompositionConflictingTypes : public std::runtime_error
{
  explicit ParameterCompositionConflictingTypes(std::string const & parameter_name)
  : std::runtime_error(
      std::string(
        "ParamMangement | The following parameter is defined with multiple different types: ") +
      parameter_name + " | This is an implementation issue.")
  {
  }
};
struct ParameterCompositionConflictingValues : public std::runtime_error
{
  explicit ParameterCompositionConflictingValues(std::string const & parameter_name)
  : std::runtime_error(
      std::string("ParamMangement | The following parameter is defined with multiple different "
                  "default values: ") +
      parameter_name + " | This is an implementation issue.")
  {
  }
};
};  // namespace exceptions

enum class ParameterType {
  // NOT_SET = 0,  // Won't be supported even though ros2 in theory supports it
  BOOL = 1,
  INTEGER = 2,
  DOUBLE = 3,
  STRING = 4,
  BYTE_ARRAY = 5,
  BOOL_ARRAY = 6,
  INTEGER_ARRAY = 7,
  DOUBLE_ARRAY = 8,
  STRING_ARRAY = 9,
};

#define TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(MACRO)    \
  MACRO(BOOL, bool, bool)                                    \
  MACRO(INTEGER, std::int64_t, int)                          \
  MACRO(DOUBLE, double, double)                              \
  MACRO(STRING, std::string, string)                         \
  MACRO(BYTE_ARRAY, std::vector<std::uint8_t>, byte_array)   \
  MACRO(BOOL_ARRAY, std::vector<bool>, bool_array)           \
  MACRO(INTEGER_ARRAY, std::vector<std::int64_t>, int_array) \
  MACRO(DOUBLE_ARRAY, std::vector<double>, double_array)     \
  MACRO(STRING_ARRAY, std::vector<std::string>, string_array)

typedef std::variant<
  bool, std::int64_t, double, std::string, std::vector<std::uint8_t>, std::vector<bool>,
  std::vector<std::int64_t>, std::vector<double>, std::vector<std::string>>
  param_value_variant_t;
typedef std::variant<
  bool *, std::int64_t *, double *, std::string *, std::vector<std::uint8_t> *, std::vector<bool> *,
  std::vector<std::int64_t> *, std::vector<double> *, std::vector<std::string> *>
  param_ptr_variant_t;
struct ParameterValueComparisonResult
{
  bool type_equal, value_equal;
  ParameterValueComparisonResult(bool type_equal_, bool value_equal_)
  : type_equal(type_equal_), value_equal(value_equal_)
  {
  }
  explicit operator bool() { return type_equal && value_equal; }
};
class ParameterValue
{
private:
  ParameterType parameter_type;
  param_value_variant_t value;

public:
  ParameterValue(const ParameterType & parameter_type_, const param_value_variant_t & value_)
  : parameter_type{parameter_type_}
  {
    set_value(value_);
  }
  void set_value(const param_value_variant_t & value_)
  {
    // Check for correct input type
    bool correct_type = false;
// clang-format off
    #define CHECK_TYPE(enum_val, typename, name)                 \
      case ParameterType::enum_val: correct_type = std::holds_alternative<typename>(value_); break;

    switch (parameter_type) {
      TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(CHECK_TYPE)
      default: throw exceptions::UnsupportedParameterType(static_cast<int>(parameter_type)); break;
    }
    #undef CHECK_TYPE
    // clang-format on
    if (!correct_type) {
      throw exceptions::BadParameterSet();
    }
    value = value_;
  }
  ParameterType get_type() const { return parameter_type; }
// clang-format off
  #define TYPE_CONVERSION_GETTER(enum_val, typename, name) \
    typename as_##name() const { \
      if (ParameterType::enum_val != parameter_type) {throw exceptions::BadParameterCast();} \
      return std::get<typename>(value);}

  TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(TYPE_CONVERSION_GETTER)

  #undef TYPE_CONVERSION_GETTER
  // clang-format on
  ParameterValueComparisonResult operator==(ParameterValue const & other)
  {
    if (parameter_type != other.parameter_type) {
      return {false, false};
    }
    // Handle floating point types with special care
    if (parameter_type == ParameterType::DOUBLE) {
      return {
        true, std::abs(as_double() - other.as_double()) <= std::numeric_limits<double>::epsilon()};
    }
    if (parameter_type == ParameterType::DOUBLE_ARRAY) {
      auto a = as_double_array();
      auto b = other.as_double_array();
      // Check length
      if (a.size() != b.size()) {
        return {true, false};
      }
      // Check if every element of the vectors is float equal
      bool all_equal = true;
      for (std::size_t i = 0; i < a.size(); ++i) {
        all_equal = all_equal && (std::abs(a[i] - b[i]) <= std::numeric_limits<double>::epsilon());
      }
      return {true, all_equal};
    }
    // Handle all remaining types
    return {true, value == other.value};
  }
};
class ParameterReference
{
private:
  ParameterType parameter_type;
  param_ptr_variant_t value_ptr;

public:
  ParameterReference(
    ParameterType const & parameter_type_, param_ptr_variant_t & value_ptr_,
    param_value_variant_t const & default_value)
  : parameter_type{parameter_type_}, value_ptr{value_ptr_}
  {
    set_value(default_value);
  }
  void set_value(param_value_variant_t const & value_) const
  {
// clang-format off
    #define SET_TYPE_SAFE(enum_val, typename, name) \
      case ParameterType::enum_val: \
      if(!std::holds_alternative<typename *>(value_ptr) || \
         !std::holds_alternative<typename>(value_)) { \
        throw exceptions::BadParameterSet(); \
      } \
        *(std::get<typename *>(value_ptr)) = std::get<typename>(value_); \
        break;

    // Do this switch here to ensure type safety on set
    switch (parameter_type) {
      TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(SET_TYPE_SAFE)
      default: throw exceptions::UnsupportedParameterType(static_cast<int>(parameter_type)); break;
    }
    #undef SET_TYPE_SAFE
    // clang-format on
  }
  ParameterValue to_value() const
  {
// clang-format off
    #define CREATE_PARAM_VALUE(enum_val, typename, name) \
      case ParameterType::enum_val: \
        return ParameterValue(parameter_type, *(std::get<typename *>(value_ptr))); \
        /* Should be type safe since the ptr is guaranteed to be of the correct
        type bc of the set value function */

    switch (parameter_type) {
      TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(CREATE_PARAM_VALUE)
    }
    throw exceptions::UnsupportedParameterType(static_cast<int>(parameter_type));
    // clang-format on
  }
  ParameterType get_type() const { return parameter_type; }
};  // namespace tam::pmg
}  // namespace tam::pmg
