// Copyright 2024 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <variant>
#include <vector>

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  m.def("init_from_yaml", &tam::pmg::init_from_yaml);
  m.def(
    "load_overwrites_from_yaml",
    py::overload_cast<
      tam::pmg::MgmtInterface *, const std::string &, const std::string &, bool, bool>(
      &tam::pmg::load_overwrites_from_yaml));
}
