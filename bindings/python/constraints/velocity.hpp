#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/OmniWheels4X.h>

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
        .def("getDT", &VelocityLimits::getDT)
        .def("update", &VelocityLimits::update);
}

void pyVelocityOmniWheels4X(py::module& m) {
    py::class_<OmniWheels4X, std::shared_ptr<OmniWheels4X>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "OmniWheels4X")
        .def(py::init<const double, const double, const double, const std::vector<std::string>, const std::string, XBot::ModelInterface&>())
        .def("update", &OmniWheels4X::update)
        .def("setIsGlobalVelocity", &OmniWheels4X::setIsGlobalVelocity)
        .def("getIsGlobalVelocity", &OmniWheels4X::getIsGlobalVelocity);

}
