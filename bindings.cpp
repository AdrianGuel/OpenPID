#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h> 
#include "pid.hpp"
#include "state_feedback.hpp"
#include "generic_system.hpp"
#include "generic_controller.hpp"

namespace py = pybind11;
using GenericSystemF = GenericSystem<float>;
using GenericControllerF = GenericController<float>;

PYBIND11_MODULE(openpid, m) {
    py::class_<PID<float>>(m, "PID")
    .def(py::init<float, float, float>())
    .def("reset", &PID<float>::reset)
    .def("compute", &PID<float>::compute)
    .def("compute_recursive", &PID<float>::compute_recursive); 

    py::class_<GenericSystemF>(m, "GenericSystem")
        .def(py::init([](py::function f, const Eigen::VectorXf& x0) {
            auto wrapper = [f](const Eigen::VectorXf& x, const Eigen::VectorXf& u) {
                return f(x, u).cast<Eigen::VectorXf>();
            };
            return new GenericSystemF(wrapper, x0);
        }))
        .def("reset", &GenericSystemF::reset)
        .def("update", &GenericSystemF::update)
        .def("get_state", &GenericSystemF::get_state);

    py::class_<StateFeedback<float, 4>>(m, "StateFeedback")
        .def(py::init<const Eigen::Matrix<float, 4, 1>&>())
        .def("set_reference", &StateFeedback<float, 4>::set_reference)
        .def("compute", &StateFeedback<float, 4>::compute);

    py::class_<GenericControllerF>(m, "GenericController")
        .def(py::init([](py::function f) {
            auto wrapper = [f](const Eigen::VectorXf& x, const Eigen::VectorXf& r) {
                return f(x, r).cast<float>();
            };
            return new GenericControllerF(wrapper);
        }))
        .def("set_reference", &GenericControllerF::set_reference)
        .def("compute", &GenericControllerF::compute);
}
