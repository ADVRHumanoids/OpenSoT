#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/acceleration/Postural.h>

namespace py = pybind11;
using namespace OpenSoT::tasks;

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> postural_get_reference_(const acceleration::Postural& postural)
{
    Eigen::VectorXd qref, dqref, ddqref;
    postural.getReference(qref, dqref, ddqref);
    return std::make_tuple(qref, dqref, ddqref);
}


void pyAccelerationPostural(py::module& m) {
    py::class_<acceleration::Postural, std::shared_ptr<acceleration::Postural>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "Postural")
        .def(py::init<const XBot::ModelInterface&, AffineHelper, const std::string&>(),
             py::arg(), py::arg(), py::arg("task_id") = "Postural")
        .def(py::init<const XBot::ModelInterface&, const std::string&>(),
             py::arg(), py::arg("task_id") = "Postural")
        .def("setGainType", &acceleration::Postural::setGainType)
        .def("getGainType", &acceleration::Postural::getGainType)
        .def("setReference", py::overload_cast<const Eigen::VectorXd&>(&acceleration::Postural::setReference))
        .def("setReference", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&>(&acceleration::Postural::setReference))
        .def("setReference", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&>(&acceleration::Postural::setReference))
        .def("getReference", postural_get_reference_)
        .def("getActualPositions", &acceleration::Postural::getActualPositions)
        .def("getError", &acceleration::Postural::getError)
        .def("getVelocityError", &acceleration::Postural::getVelocityError)
        .def("setLambda", py::overload_cast<double, double>(&acceleration::Postural::setLambda))
        .def("setLambda", py::overload_cast<double>(&acceleration::Postural::setLambda))
        .def("getLambda2", &acceleration::Postural::getLambda2)
        .def("reset", &acceleration::Postural::reset)
        .def("getCachedVelocityReference", &acceleration::Postural::getCachedVelocityReference)
        .def("getCachedAccelerationReference", &acceleration::Postural::getCachedAccelerationReference)
        .def("setKp", &acceleration::Postural::setKp)
        .def("setKd", &acceleration::Postural::setKd)
        .def("setGains", &acceleration::Postural::setGains)
        .def("getKp", &acceleration::Postural::getKp)
        .def("getKd", &acceleration::Postural::getKd);
}
