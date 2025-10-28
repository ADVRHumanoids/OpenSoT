#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>


namespace py = pybind11;
using namespace OpenSoT::constraints::velocity;



using CollisionAvoidanceC = OpenSoT::constraints::velocity::CollisionAvoidance;

std::shared_ptr<CollisionAvoidanceC> make_collision_avoidance(
    const XBot::ModelInterface& model, int max_pairs, 
    std::string collision_urdf_str, std::string collision_srdf_str)
{
    urdf::ModelSharedPtr collision_urdf;
    if(!collision_urdf_str.empty()){
        collision_urdf = std::make_shared<urdf::Model>();
        if(!collision_urdf->initString(collision_urdf_str)){
            throw std::runtime_error("Failed to parse collision_urdf string");
        }
    }

    srdf::ModelSharedPtr collision_srdf;
    if(!collision_srdf_str.empty()){
        collision_srdf = std::make_shared<srdf::Model>();
        urdf::ModelConstSharedPtr active_urdf = collision_urdf ? collision_urdf : model.getUrdf();
        if(!collision_srdf->initString(*active_urdf, collision_srdf_str)){
            throw std::runtime_error("Failed to parse collision_srdf string");
        }
    }

    return std::make_shared<CollisionAvoidanceC>(model, max_pairs, collision_urdf, collision_srdf);
}


std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> get_ordered_witness_point_vector(const CollisionAvoidanceC& ca)
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ordered_witness_points;
    ca.getOrderedWitnessPointVector(ordered_witness_points);
    return ordered_witness_points;
}


void pyVelocityCollisionAvoidance(py::module& m) {
    
    py::class_<CollisionAvoidanceC, std::shared_ptr<CollisionAvoidanceC>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CollisionAvoidance")
        .def(py::init(&make_collision_avoidance),
             py::arg(), py::arg("max_pairs") = -1, py::arg("collision_urdf") = "", py::arg("collision_srdf") = "")
        .def("getLinkPairThreshold", &CollisionAvoidanceC::getLinkPairThreshold)
        .def("getDetectionThreshold", &CollisionAvoidanceC::getDetectionThreshold)
        .def("setLinkPairThreshold", &CollisionAvoidanceC::setLinkPairThreshold)
        .def("setDetectionThreshold", &CollisionAvoidanceC::setDetectionThreshold)
        .def("update", &CollisionAvoidanceC::update)
        .def("setMaxPairs", &CollisionAvoidanceC::setMaxPairs)
        .def("setCollisionList", &CollisionAvoidanceC::setCollisionList)
        .def("collisionModelUpdated", &CollisionAvoidanceC::collisionModelUpdated)
        .def("addCollisionShape", &CollisionAvoidanceC::addCollisionShape)
        .def("setCollisionShapeActive", &CollisionAvoidanceC::setCollisionShapeActive)
        .def("moveCollisionShape", &CollisionAvoidanceC::moveCollisionShape)
        .def("setBoundScaling", &CollisionAvoidanceC::setBoundScaling)
        .def("setLinksVsEnvironment", &CollisionAvoidanceC::setLinksVsEnvironment)
        .def("getCollisionModel", (const XBot::Collision::CollisionModel& (CollisionAvoidanceC::*)() const) &CollisionAvoidanceC::getCollisionModel)
        .def("getOrderedWitnessPointVector", &get_ordered_witness_point_vector)
        .def("getOrderedLinkPairVector", &CollisionAvoidanceC::getOrderedLinkPairVector)
        .def("getOrderedDistanceVector", &CollisionAvoidanceC::getOrderedDistanceVector)
        .def("getCollisionJacobian", &CollisionAvoidanceC::getCollisionJacobian);
}
