#include <pybind11/pybind11.h>
#include "pid.hpp"
#include "msd_system.hpp"
#include "pendulum_cart.hpp" 

namespace py = pybind11;

PYBIND11_MODULE(openpid, m) {
    py::class_<PID<float>>(m, "PID")
    .def(py::init<float, float, float>())
    .def("reset", &PID<float>::reset)
    .def("compute", &PID<float>::compute)
    .def("compute_recursive", &PID<float>::compute_recursive); 

    py::class_<MassSpringDamper<float>>(m, "MassSpringDamper")
        .def(py::init<float, float, float>())
        .def("reset", &MassSpringDamper<float>::reset)
        .def("update", &MassSpringDamper<float>::update)
        .def("get_position", &MassSpringDamper<float>::get_position)
        .def("get_velocity", &MassSpringDamper<float>::get_velocity);

        py::class_<PendulumOnCart<float>>(m, "PendulumOnCart")
        .def(py::init<float, float, float, float, float, float>(),  // ← acepta 6 argumentos
             py::arg("mass_cart"), py::arg("mass_pend"), py::arg("length"),
             py::arg("friction_cart"), py::arg("inertia"), py::arg("gravity") = 9.81)
        .def("reset", &PendulumOnCart<float>::reset,
                py::arg("x0") = 0.0f, py::arg("v0") = 0.0f, py::arg("theta0") = float(M_PI + 0.01), py::arg("omega0") = 0.0f)           
        .def("update", &PendulumOnCart<float>::update)
        .def("get_position", &PendulumOnCart<float>::get_position)
        .def("get_velocity", &PendulumOnCart<float>::get_velocity)
        .def("get_angle", &PendulumOnCart<float>::get_angle)
        .def("get_angular_velocity", &PendulumOnCart<float>::get_angular_velocity);   
}
