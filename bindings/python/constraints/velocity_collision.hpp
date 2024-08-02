#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>


namespace py = pybind11;
using namespace OpenSoT::constraints::velocity;

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
