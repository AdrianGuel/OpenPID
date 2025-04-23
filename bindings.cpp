#include <pybind11/pybind11.h>
#include "pid.hpp"
#include "msd_system.hpp"

namespace py = pybind11;

PYBIND11_MODULE(openpid, m) {
    py::class_<PID<float>>(m, "PID")
        .def(py::init<float, float, float>())
        .def("reset", &PID<float>::reset)
        .def("compute", &PID<float>::compute);

    py::class_<MassSpringDamper<float>>(m, "MassSpringDamper")
        .def(py::init<float, float, float>())
        .def("reset", &MassSpringDamper<float>::reset)
        .def("update", &MassSpringDamper<float>::update)
        .def("get_position", &MassSpringDamper<float>::get_position)
        .def("get_velocity", &MassSpringDamper<float>::get_velocity);
}
