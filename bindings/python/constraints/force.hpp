#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/constraints/force/CoP.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/force/NormalTorque.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>

namespace py = pybind11;
using namespace OpenSoT::constraints::force;

void pyForceCoP(py::module& m) {
    py::class_<CoP, std::shared_ptr<CoP>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CoP")
        .def(py::init<const std::string&, const AffineHelper&, XBot::ModelInterface&, const Eigen::Vector2d&, const Eigen::Vector2d&>())
        .def("update", &CoP::update);

    py::class_<CoPs, std::shared_ptr<CoPs>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CoPs")
        .def(py::init<const std::vector<AffineHelper>&, const std::vector<std::string>&, XBot::ModelInterface&, const std::vector<Eigen::Vector2d>&, const std::vector<Eigen::Vector2d>&>())
        .def("update", &CoPs::update);
}

void pyForceFrictionCone(py::module& m) {
    py::class_<FrictionCone, std::shared_ptr<FrictionCone>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "FrictionCone")
        .def(py::init<const std::string&, const AffineHelper&, XBot::ModelInterface&, const std::pair<Eigen::Matrix3d, double>&>())
        .def("update", &FrictionCone::update)
        .def("setFrictionCone", &FrictionCone::setFrictionCone)
        .def("setMu", &FrictionCone::setMu)
        .def("setContactRotationMatrix", &FrictionCone::setContactRotationMatrix);

    py::class_<FrictionCones, std::shared_ptr<FrictionCones>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "FrictionCones")
        .def(py::init<const std::vector<std::string>&, const std::vector<AffineHelper>&, XBot::ModelInterface&, const FrictionCones::friction_cones&>())
        .def("getFrictionCone", &FrictionCones::getFrictionCone)
        .def("update", &FrictionCones::update);
}

void pyForceNormalTorque(py::module& m) {
    py::class_<NormalTorque, std::shared_ptr<NormalTorque>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "NormalTorque")
        .def(py::init<const std::string&, const AffineHelper&, XBot::ModelInterface&, const Eigen::Vector2d&, const Eigen::Vector2d&, const double&>())
        .def("update", &NormalTorque::update)
        .def("setMu", &NormalTorque::setMu);

    py::class_<NormalTorques, std::shared_ptr<NormalTorques>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "NormalTorques")
        .def(py::init<const std::vector<std::string>&, const std::vector<AffineHelper>&, XBot::ModelInterface&, const std::vector<Eigen::Vector2d>&, const std::vector<Eigen::Vector2d>&, const std::vector<double>&>())
        .def("getNormalTorque", &NormalTorques::getNormalTorque)
        .def("update", &NormalTorques::update);
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> get_wrench_limits(const WrenchLimits& wlims)
{
    Eigen::VectorXd wmin, wmax;
    wlims.getWrenchLimits(wmin, wmax);
    return std::make_tuple(wmin, wmax);
}

void pyWrenchLimits(py::module& m) {
    py::class_<WrenchLimits, std::shared_ptr<WrenchLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "WrenchLimits")
        .def(py::init<const std::string&, const Eigen::VectorXd&, const Eigen::VectorXd&, AffineHelper>())
        .def("getWrenchLimits", get_wrench_limits)
        .def("setWrenchLimits", &WrenchLimits::setWrenchLimits)
        .def("releaseContact", &WrenchLimits::releaseContact)
        .def("isReleased", &WrenchLimits::isReleased);

    py::class_<WrenchesLimits, std::shared_ptr<WrenchesLimits>, OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>>(m, "WrenchesLimits")
        .def(py::init<const std::vector<std::string>&, const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<AffineHelper>&>())
        .def(py::init<const std::vector<std::string>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<AffineHelper>&>())
        .def(py::init<const std::map<std::string, WrenchLimits::Ptr>&, const std::vector<AffineHelper>&>())
        .def("getWrenchLimits", &WrenchesLimits::getWrenchLimits)
        .def("update", &WrenchesLimits::update);
}
