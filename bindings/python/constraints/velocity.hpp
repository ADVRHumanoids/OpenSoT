#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

namespace py = pybind11;
using namespace OpenSoT::constraints::velocity;

void pyVelocityJointLimits(py::module& m) {
    py::class_<JointLimits, std::shared_ptr<JointLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "JointLimits")
        .def(py::init<const XBot::ModelInterface&, const Eigen::VectorXd&, const Eigen::VectorXd&, const double>(),
             py::arg(), py::arg(), py::arg(), py::arg("boundScaling") = 1.)
        .def("update", &JointLimits::update)
        .def("setBoundScaling", &JointLimits::setBoundScaling);
}

void pyVelocityLimits(py::module& m) {
py::class_<VelocityLimits, std::shared_ptr<VelocityLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "VelocityLimits")
        .def(py::init<const XBot::ModelInterface&, const double, const double>())
        .def(py::init<const XBot::ModelInterface&, const Eigen::VectorXd&, const double>())
        .def("getVelocityLimits", &VelocityLimits::getVelocityLimits)
        .def("setVelocityLimits", py::overload_cast<const double>(&VelocityLimits::setVelocityLimits))
        .def("setVelocityLimits", py::overload_cast<const Eigen::VectorXd&>(&VelocityLimits::setVelocityLimits))
        .def("getDT", &VelocityLimits::getDT);
}
