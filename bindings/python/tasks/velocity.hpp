#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>

namespace py = pybind11;
using namespace OpenSoT::tasks::velocity;

std::tuple<Eigen::VectorXd, Eigen::VectorXd> postural_get_reference(const Postural& postural)
{
    Eigen::VectorXd qref, dqref;
    postural.getReference(qref, dqref);
    return std::make_tuple(qref, dqref);
}

void pyVelocityPostural(py::module& m) {
    py::class_<Postural, std::shared_ptr<Postural>, Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "Postural")
        .def(py::init<XBot::ModelInterface&, const std::string&>(), py::arg(), py::arg("task_id") = "Postural")
        .def("setReference", py::overload_cast<const Eigen::VectorXd&>(&Postural::setReference))
        .def("setReference", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&>(&Postural::setReference))
        .def("getReference", postural_get_reference)
        .def("getCachedVelocityReference", &Postural::getCachedVelocityReference)
        .def("getActualPositions", &Postural::getActualPositions)
        .def("setLambda", &Postural::setLambda)
        .def("getError", &Postural::getError)
        .def("reset", &Postural::reset);
}

std::tuple<Eigen::Affine3d, Eigen::Vector6d> cartesian_get_reference(const Cartesian& cartesian)
{
    Eigen::Affine3d pose_ref;
    Eigen::Vector6d vel_ref;
    cartesian.getReference(pose_ref, vel_ref);
    return std::make_tuple(pose_ref, vel_ref);
}

void pyVelocityCartesian(py::module& m) {
    py::class_<Cartesian, std::shared_ptr<Cartesian>, Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "Cartesian")
          .def(py::init<std::string, XBot::ModelInterface&, const std::string&, const std::string&>())
          .def("setReference", py::overload_cast<const Eigen::Affine3d&>(&Cartesian::setReference))
          .def("setReference", py::overload_cast<const Eigen::Affine3d&, const Eigen::Vector6d&>(&Cartesian::setReference))
          .def("getReference", cartesian_get_reference)
          .def("getCachedVelocityReference", &Cartesian::getCachedVelocityReference)
          .def("getActualPose", py::overload_cast<>(&Cartesian::getActualPose, py::const_))
          .def("getError", &Cartesian::getError)
          .def("reset", &Cartesian::reset)
          .def("setIsBodyJacobian", &Cartesian::setIsBodyJacobian)
          .def_property("orientationErrorGain", &Cartesian::getOrientationErrorGain, &Cartesian::setOrientationErrorGain)
          .def_property_readonly("distalLink", &Cartesian::getDistalLink)
          .def_property_readonly("baseLink", &Cartesian::getBaseLink)
          .def_property_readonly("baseLinkIsWorld", &Cartesian::baseLinkIsWorld);
}


