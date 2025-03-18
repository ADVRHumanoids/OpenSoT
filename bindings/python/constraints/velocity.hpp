#include <OpenSoT/constraints/velocity/AccelerationLimits.h>
#include <OpenSoT/constraints/velocity/JerkLimits.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/OmniWheels4X.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace OpenSoT::constraints::velocity;

void pyVelocityJointLimits(py::module &m)
{
    py::class_<JointLimits, std::shared_ptr<JointLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(
        m, "JointLimits")
        .def(py::init<const XBot::ModelInterface &, const Eigen::VectorXd &, const Eigen::VectorXd &, const double>(),
             py::arg(), py::arg(), py::arg(), py::arg("boundScaling") = 1.)
        .def("update", &JointLimits::update)
        .def("setBoundScaling", &JointLimits::setBoundScaling);
}

void pyVelocityLimits(py::module &m)
{
    py::class_<VelocityLimits, std::shared_ptr<VelocityLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(
        m, "VelocityLimits")
        .def(py::init<const XBot::ModelInterface &, const double, const double>())
        .def(py::init<const XBot::ModelInterface &, const Eigen::VectorXd &, const double>())
        .def("getVelocityLimits", &VelocityLimits::getVelocityLimits)
        .def("setVelocityLimits", py::overload_cast<const double>(&VelocityLimits::setVelocityLimits))
        .def("setVelocityLimits", py::overload_cast<const Eigen::VectorXd &>(&VelocityLimits::setVelocityLimits))
        .def("getDT", &VelocityLimits::getDT)
        .def("update", &VelocityLimits::update);
}

void pyVelocityAccelerationLimits(py::module &m)
{
    py::class_<AccelerationLimits, std::shared_ptr<AccelerationLimits>,
               OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AccelerationLimits")
        .def(py::init<const XBot::ModelInterface &, const Eigen::VectorXd &, const double>(),
             py::arg("model"), py::arg("qDDotLimit"), py::arg("dT"))
        .def("getLowerBound", &AccelerationLimits::getLowerBound)
        .def("getUpperBound", &AccelerationLimits::getUpperBound)
        .def("getAccelerationLimits", &AccelerationLimits::getAccelerationLimits)
        .def("setAccelerationLimits",
             py::overload_cast<const Eigen::VectorXd &>(&AccelerationLimits::setAccelerationLimits))
        .def("getDT", &AccelerationLimits::getDT)
        .def("update", &AccelerationLimits::update);
}

void pyVelocityJerkLimits(py::module &m)
{
    py::class_<JerkLimits, std::shared_ptr<JerkLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(
        m, "JerkLimits")
        .def(py::init<const XBot::ModelInterface &, const Eigen::VectorXd &, const double>(),
             py::arg("model"), py::arg("qDDDotLimit"), py::arg("dT"))
        .def("getLowerBound", &JerkLimits::getLowerBound)
        .def("getUpperBound", &JerkLimits::getUpperBound)
        .def("getJerkLimits", &JerkLimits::getJerkLimits)
        .def("setJerkLimits", py::overload_cast<const Eigen::VectorXd &>(&JerkLimits::setJerkLimits))
        .def("getDT", &JerkLimits::getDT)
        .def("update", &JerkLimits::update);
}

void pyVelocityOmniWheels4X(py::module &m)
{
    py::class_<OmniWheels4X, std::shared_ptr<OmniWheels4X>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(
        m, "OmniWheels4X")
        .def(py::init<const double, const double, const double, const std::vector<std::string>, const std::string,
                      XBot::ModelInterface &>())
        .def("update", &OmniWheels4X::update)
        .def("setIsGlobalVelocity", &OmniWheels4X::setIsGlobalVelocity)
        .def("getIsGlobalVelocity", &OmniWheels4X::getIsGlobalVelocity);
}
