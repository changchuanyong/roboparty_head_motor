// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#include <pybind11/pybind11.h>

#include "head_controller.hpp"

namespace py = pybind11;

PYBIND11_MODULE(head_motor_py, m) {
    m.doc() = "Head Controller Python SDK (two-axis LRO head motors)";

    py::class_<HeadController>(m, "HeadController")
        .def(py::init<const std::string&, uint16_t, uint16_t>(),
             py::arg("can_interface"),
             py::arg("yaw_id") = 1,
             py::arg("pitch_id") = 2)
        // Lifecycle
        .def("init_motors", &HeadController::init_motors)
        .def("deinit_motors", &HeadController::deinit_motors)
        .def("enable", &HeadController::enable)
        .def("disable", &HeadController::disable)
        // Motion control (non-blocking, speed-limited approach by the control thread)
        .def("move", &HeadController::move,
             py::arg("axis"), py::arg("deg"), py::arg("speed") = 30.0f)
        .def("look_at", &HeadController::look_at,
             py::arg("yaw_deg"), py::arg("pitch_deg"),
             py::arg("yaw_speed") = 30.0f, py::arg("pitch_speed") = 30.0f)
        // Queries
        .def("get_joint_deg", &HeadController::get_joint_deg, py::arg("axis"))
        .def("get_temp", &HeadController::get_temp, py::arg("axis"))
        .def("get_error", &HeadController::get_error, py::arg("axis"))
        // Configuration
        .def("set_limits", &HeadController::set_limits,
             py::arg("axis"), py::arg("min_deg"), py::arg("max_deg"))
        .def("set_zero", &HeadController::set_zero, py::arg("axis"))
        .def("clear_error", &HeadController::clear_error, py::arg("axis"))
        // Axis constants (1 = yaw left/right, 2 = pitch up/down)
        .def_readonly_static("AXIS_YAW", &HeadController::AXIS_YAW)
        .def_readonly_static("AXIS_PITCH", &HeadController::AXIS_PITCH);
}
