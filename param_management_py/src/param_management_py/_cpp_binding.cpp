// Copyright 2024 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <variant>
#include <vector>

#include "param_management_cpp/base.hpp"
#include "param_management_cpp/init_backend.hpp"
#include "param_management_cpp/initialization.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
// clang-format off
  #define BIND_ENUM_MEMBER(enum_val, typename, name) \
    .value(#enum_val, tam::pmg::ParameterType::enum_val)
  #define BIND_TYPE_CONVERSION_GETTER(enum_val, typename, name) \
    .def("as_" #name, &tam::pmg::ParameterValue::as_##name, "Get parameter value as " #name)
  py::enum_<tam::pmg::ParameterType>(m, "ParameterType")
    TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(BIND_ENUM_MEMBER);
  py::class_<tam::pmg::ParameterValue>(m, "ParameterValue")
    .def(py::init<tam::pmg::ParameterType, tam::pmg::param_value_variant_t>())
    .def("set_value", &tam::pmg::ParameterValue::set_value, "Set a parameter")
    .def("get_type", &tam::pmg::ParameterValue::get_type, "Get the type of the parameter")
    TAM_PMG_FOR_EVERY_SUPPORTED_PARAMETER_TYPE(BIND_TYPE_CONVERSION_GETTER);
  py::class_<tam::pmg::backend::InitializationBackend>(m, "InitializationBackend")
    .def(py::init())
    .def("clear", &tam::pmg::backend::InitializationBackend::clear)
    .def("get_init_val", &tam::pmg::backend::InitializationBackend::get_init_val)
    .def("set_init_val", &tam::pmg::backend::InitializationBackend::set_init_val)
    .def("has_init_val", &tam::pmg::backend::InitializationBackend::has_init_val);
  m.def(
    "init", &tam::pmg::init,
    "Set initial values for parameters, that are used instead of the default values for "
    "declaration");
  m.def("reset", &tam::pmg::reset, "Reset the initialization layer");
  py::class_<tam::pmg::MgmtInterface, tam::pmg::MgmtInterface::SharedPtr>(m, "MgmtInterface")
    .def("has_parameter", &tam::pmg::MgmtInterface::has_parameter)
    .def("list_parameters", &tam::pmg::MgmtInterface::list_parameters)
    .def("get_type", &tam::pmg::MgmtInterface::get_type)
    .def("get_description", &tam::pmg::MgmtInterface::get_description)
    .def("set_value", &tam::pmg::MgmtInterface::set_value)
    .def("get_value", &tam::pmg::MgmtInterface::get_value);
  py::class_<tam::pmg::UserInterface, tam::pmg::UserInterface::SharedPtr>(m, "UserInterface")
    .def("declare_parameter", &tam::pmg::UserInterface::declare_parameter, "Declare a parameter")
    .def("get_value", &tam::pmg::UserInterface::get_value, "Get a parameter value")
    .def(
      "declare_and_get_value", &tam::pmg::UserInterface::declare_and_get_value,
      "Declare a parameter if it not declared and get the parameters value")
    .def("get_state_hash", &tam::pmg::UserInterface::get_state_hash, "Get a state hash");
  py::class_<
    tam::pmg::ParamValueManager, tam::pmg::MgmtInterface, tam::pmg::UserInterface,
    tam::pmg::ParamValueManager::SharedPtr>(m, "ParamValueManager")
    .def(py::init());
  py::class_<
    tam::pmg::ParamManagerComposer, tam::pmg::MgmtInterface,
    tam::pmg::ParamManagerComposer::SharedPtr>(m, "ParamManagerComposer")
    .def(py::init<std::vector<tam::pmg::MgmtInterface::SharedPtr>>());
  // Bind exceptions
  py::register_exception<tam::pmg::exceptions::NonDeclaredParameterAccess>(
    m, "NonDeclaredParameterAccess");
  py::register_exception<tam::pmg::exceptions::BadParameterType>(m, "BadParameterType");
  py::register_exception<tam::pmg::exceptions::BadParameterSet>(m, "BadParameterSet");
  py::register_exception<tam::pmg::exceptions::BadParameterCast>(m, "BadParameterCast");
  py::register_exception<tam::pmg::exceptions::ParameterCompositionConflictingTypes>(
    m, "ParameterCompositionConflictingTypes");
  py::register_exception<tam::pmg::exceptions::ParameterCompositionConflictingValues>(
    m, "ParameterCompositionConflictingValues");
  py::register_exception<tam::pmg::exceptions::RedeclarationOfExistingParameter>(
    m, "RedeclarationOfExistingParameter");
  py::register_exception<tam::pmg::exceptions::UnsupportedParameterType>(
    m, "UnsupportedParameterType");
}
