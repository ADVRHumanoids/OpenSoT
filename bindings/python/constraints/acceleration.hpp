#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/acceleration/JointLimits.h>
#include <OpenSoT/constraints/acceleration/TorqueLimits.h>
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>


namespace py = pybind11;

void pyAccelerationJointLimits(py::module& m) {
    py::class_<OpenSoT::constraints::acceleration::JointLimits, std::shared_ptr<OpenSoT::constraints::acceleration::JointLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "JointLimits")
        .def(py::init<XBot::ModelInterface&, const AffineHelper&, const Eigen::VectorXd&,
                      const Eigen::VectorXd&, const Eigen::VectorXd&, const double>())
        .def("update", &OpenSoT::constraints::acceleration::JointLimits::update)
        .def("setJointAccMax", &OpenSoT::constraints::acceleration::JointLimits::setJointAccMax)
        .def("setPStepAheadPredictor", &OpenSoT::constraints::acceleration::JointLimits::setPStepAheadPredictor);
}

void pyTorqueLimits(py::module& m) {
    py::class_<OpenSoT::constraints::acceleration::TorqueLimits, std::shared_ptr<OpenSoT::constraints::acceleration::TorqueLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "TorqueLimits")
            .def(py::init<const XBot::ModelInterface&, const AffineHelper&, const std::vector<AffineHelper>&,
                          const std::vector<std::string>&, const Eigen::VectorXd&>())
            .def("update", &OpenSoT::constraints::acceleration::TorqueLimits::update)
            .def("setTorqueLimits", &OpenSoT::constraints::acceleration::TorqueLimits::setTorqueLimits)
            .def("enableContact", &OpenSoT::constraints::acceleration::TorqueLimits::enableContact)
            .def("disableContact", &OpenSoT::constraints::acceleration::TorqueLimits::disableContact)
            .def("getEnabledContacts", &OpenSoT::constraints::acceleration::TorqueLimits::getEnabledContacts);
}

void pyAVelocityLimits(py::module& m) {
    py::class_<OpenSoT::constraints::acceleration::VelocityLimits, std::shared_ptr<OpenSoT::constraints::acceleration::VelocityLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "VelocityLimits")
            .def(py::init<XBot::ModelInterface&, const AffineHelper&, const double, const double>())
            .def(py::init<XBot::ModelInterface&, const AffineHelper&, const Eigen::VectorXd&, const double>())
            .def("update", &OpenSoT::constraints::acceleration::VelocityLimits::update)
            .def("setVelocityLimits", py::overload_cast<const double>(&OpenSoT::constraints::acceleration::VelocityLimits::setVelocityLimits))
            .def("setVelocityLimits", py::overload_cast<const Eigen::VectorXd&>(&OpenSoT::constraints::acceleration::VelocityLimits::setVelocityLimits))
            .def("setPStepAheadPredictor", &OpenSoT::constraints::acceleration::VelocityLimits::setPStepAheadPredictor);
}
