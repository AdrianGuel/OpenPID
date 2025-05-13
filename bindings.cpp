#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h> 
#include "pid.hpp"
#include "state_feedback.hpp"
#include "generic_system.hpp"

namespace py = pybind11;

PYBIND11_MODULE(openpid, m) {
    py::class_<PID<float>>(m, "PID")
    .def(py::init<float, float, float>())
    .def("reset", &PID<float>::reset)
    .def("compute", &PID<float>::compute)
    .def("compute_recursive", &PID<float>::compute_recursive); 

    py::class_<GenericSystem<float>>(m, "GenericSystem")
        .def(py::init<GenericSystem<float>::DynamicsFunc, const Eigen::VectorXf&>())
        .def("reset", &GenericSystem<float>::reset)
        .def("update", &GenericSystem<float>::update)
        .def("get_state", &GenericSystem<float>::get_state);

    py::class_<StateFeedback<float, 4>>(m, "StateFeedback")
        .def(py::init<const Eigen::Matrix<float, 4, 1>&>())
        .def("set_reference", &StateFeedback<float, 4>::set_reference)
        .def("compute", &StateFeedback<float, 4>::compute);          
}
