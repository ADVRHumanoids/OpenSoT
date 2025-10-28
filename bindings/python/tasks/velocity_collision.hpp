#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/velocity/CollisionAvoidance.h>

namespace py = pybind11;
using namespace OpenSoT::tasks::velocity;

void pyVelocityCollisionAvoidanceTask(py::module& m) {
    using CollisionAvoidanceT = OpenSoT::tasks::velocity::CollisionAvoidance;
    using CollisionAvoidanceC = OpenSoT::constraints::velocity::CollisionAvoidance;
    py::class_<CollisionAvoidanceT, std::shared_ptr<CollisionAvoidanceT>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CollisionAvoidance")
        .def(py::init<CollisionAvoidanceC::Ptr>(), py::arg("constraint"))
        .def("getConstraint", &CollisionAvoidanceT::getConstraint);
}

