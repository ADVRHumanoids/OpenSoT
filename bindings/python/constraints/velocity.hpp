#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/OmniWheels4X.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>


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

void pyVelocityOmniWheels4X(py::module& m) {
    py::class_<OmniWheels4X, std::shared_ptr<OmniWheels4X>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "OmniWheels4X")
        .def(py::init<const double, const double, const double, const std::vector<std::string>, const std::string, XBot::ModelInterface&>());

}

void pyVelocityCollisionAvoidance(py::module& m) {
    py::class_<CollisionAvoidance, std::shared_ptr<CollisionAvoidance>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CollisionAvoidance")
        .def(py::init<const XBot::ModelInterface&, int, urdf::ModelConstSharedPtr, srdf::ModelConstSharedPtr>(),
             py::arg(), py::arg("max_pairs") = -1, py::arg("collision_urdf") = nullptr, py::arg("collision_srdf") = nullptr)
        .def("getLinkPairThreshold", &CollisionAvoidance::getLinkPairThreshold)
        .def("getDetectionThreshold", &CollisionAvoidance::getDetectionThreshold)
        .def("setLinkPairThreshold", &CollisionAvoidance::setLinkPairThreshold)
        .def("setDetectionThreshold", &CollisionAvoidance::setDetectionThreshold)
        .def("update", &CollisionAvoidance::update)
        .def("setMaxPairs", &CollisionAvoidance::setMaxPairs)
        .def("setCollisionList", &CollisionAvoidance::setCollisionList)
        .def("collisionModelUpdated", &CollisionAvoidance::collisionModelUpdated)
        .def("addCollisionShape", &CollisionAvoidance::addCollisionShape)
        .def("moveCollisionShape", &CollisionAvoidance::moveCollisionShape)
        .def("setBoundScaling", &CollisionAvoidance::setBoundScaling)
        .def("setLinksVsEnvironment", &CollisionAvoidance::setLinksVsEnvironment)
        .def("getCollisionModel", (const XBot::Collision::CollisionModel& (CollisionAvoidance::*)() const) &CollisionAvoidance::getCollisionModel)
        .def("getOrderedWitnessPointVector", &CollisionAvoidance::getOrderedWitnessPointVector)
        .def("getOrderedLinkPairVector", &CollisionAvoidance::getOrderedLinkPairVector)
        .def("getOrderedDistanceVector", &CollisionAvoidance::getOrderedDistanceVector)
        .def("getCollisionJacobian", &CollisionAvoidance::getCollisionJacobian);
}

