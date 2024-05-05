#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>

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


std::tuple<Eigen::Affine3d, Eigen::Vector6d, Eigen::Vector6d> cartesian_get_reference_(const acceleration::Cartesian& cartesian)
{
    Eigen::Affine3d pose_ref;
    Eigen::Vector6d vel_ref, acc_ref;
    cartesian.getReference(pose_ref, vel_ref, acc_ref);
    return std::make_tuple(pose_ref, vel_ref, acc_ref);
}

Eigen::Affine3d cartesian_get_actual_pose_(const acceleration::Cartesian& cartesian)
{
    Eigen::Affine3d actual_pose;
    cartesian.getActualPose(actual_pose);
    return actual_pose;
}

Eigen::Vector6d cartesian_get_actual_twist_(const acceleration::Cartesian& cartesian)
{
    Eigen::Vector6d actual_twist;
    cartesian.getActualTwist(actual_twist);
    return actual_twist;
}

void pyAccelerationCartesian(py::module& m) {
    py::class_<acceleration::Cartesian, std::shared_ptr<acceleration::Cartesian>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "Cartesian")
            .def(py::init<const std::string, const XBot::ModelInterface&, const std::string&, const std::string&>())
            .def(py::init<const std::string, const XBot::ModelInterface&, const std::string&, const std::string&, const AffineHelper&>())
            .def("setGainType", &acceleration::Cartesian::setGainType)
            .def("getGainType", &acceleration::Cartesian::getGainType)
            .def("getBaseLink", &acceleration::Cartesian::getBaseLink)
            .def("getDistalLink", &acceleration::Cartesian::getDistalLink)
            .def("setReference", py::overload_cast<const Eigen::Affine3d&>(&acceleration::Cartesian::setReference))
            .def("setReference", py::overload_cast<const Eigen::Affine3d&, const Eigen::Vector6d&>(&acceleration::Cartesian::setReference))
            .def("setReference", py::overload_cast<const Eigen::Affine3d&, const Eigen::Vector6d&, const Eigen::Vector6d&>(&acceleration::Cartesian::setReference))
            .def("setVirtualForce", &acceleration::Cartesian::setVirtualForce)
            .def("getReference", cartesian_get_reference_)
            .def("getCachedVelocityReference", &acceleration::Cartesian::getCachedVelocityReference)
            .def("getCachedAccelerationReference", &acceleration::Cartesian::getCachedAccelerationReference)
            .def("getCachedVirtualForceReference", &acceleration::Cartesian::getCachedVirtualForceReference)
            .def("getActualPose", cartesian_get_actual_pose_)
            .def("getActualTwist", cartesian_get_actual_twist_)
            .def("reset", &acceleration::Cartesian::reset)
            .def("setLambda", py::overload_cast<double, double>(&acceleration::Cartesian::setLambda))
            .def("setLambda", py::overload_cast<double>(&acceleration::Cartesian::setLambda))
            .def("getLambda2", &acceleration::Cartesian::getLambda2)
            .def("setOrientationGain", &acceleration::Cartesian::setOrientationGain)
            .def("getOrientationErrorGain", &acceleration::Cartesian::getOrientationErrorGain)
            .def("baseLinkIsWorld", &acceleration::Cartesian::baseLinkIsWorld)
            .def("setDistalLink", &acceleration::Cartesian::setDistalLink)
            .def("setBaseLink", &acceleration::Cartesian::setBaseLink)
            .def("setKp", &acceleration::Cartesian::setKp)
            .def("setKd", &acceleration::Cartesian::setKd)
            .def("setGains", &acceleration::Cartesian::setGains)
            .def("getKp", &acceleration::Cartesian::getKp)
            .def("getKd", &acceleration::Cartesian::getKd)
            .def("getError", &acceleration::Cartesian::getError)
            .def("getVelocityError", &acceleration::Cartesian::getVelocityError);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> angular_momentum_get_reference(const acceleration::AngularMomentum& am)
{
    Eigen::Vector3d l_ref, dl_ref;
    am.getReference(l_ref, dl_ref);
    return std::make_tuple(l_ref, dl_ref);
}

void pyAccelerationAngularMomentum(py::module& m) {
    py::class_<acceleration::AngularMomentum, std::shared_ptr<acceleration::AngularMomentum>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "AngularMomentum")
        .def(py::init<XBot::ModelInterface&, const AffineHelper&>())
        .def("setReference", py::overload_cast<const Eigen::Vector3d&>(&acceleration::AngularMomentum::setReference))
        .def("setReference", py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&>(&acceleration::AngularMomentum::setReference))
        .def("setMomentumGain", &acceleration::AngularMomentum::setMomentumGain)
        .def("getReference", angular_momentum_get_reference)
        .def("getBaseLink", &acceleration::AngularMomentum::getBaseLink)
        .def("getDistalLink", &acceleration::AngularMomentum::getDistalLink)
        .def("reset", &acceleration::AngularMomentum::reset);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> com_get_reference_(const acceleration::CoM& com)
{
    Eigen::Vector3d pos_ref, vel_ref, acc_ref;
    com.getReference(pos_ref, vel_ref, acc_ref);
    return std::make_tuple(pos_ref, vel_ref, acc_ref);
}

Eigen::Vector3d com_get_actual_pos(const acceleration::CoM& com)
{
    Eigen::Vector3d actual_pos;
    com.getActualPose(actual_pos);
    return actual_pos;
}

Eigen::Vector3d com_get_pos_error(const acceleration::CoM& com)
{
    Eigen::Vector3d pos_error;
    com.getPosError(pos_error);
    return pos_error;
}

void pyAccelerationCoM(py::module& m) {
    py::class_<acceleration::CoM, std::shared_ptr<acceleration::CoM>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "CoM")
            .def(py::init<const XBot::ModelInterface&>())
            .def(py::init<const XBot::ModelInterface&, const AffineHelper&>())
            .def("getBaseLink", &acceleration::CoM::getBaseLink)
            .def("getDistalLink", &acceleration::CoM::getDistalLink)
            .def("setReference", py::overload_cast<const Eigen::Vector3d&>(&acceleration::CoM::setReference))
            .def("setReference", py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&>(&acceleration::CoM::setReference))
            .def("getReference", com_get_reference_)
            .def("getActualPose", com_get_actual_pos)
            .def("getPosError", com_get_pos_error)
            .def("reset", &acceleration::CoM::reset)
            .def("setLambda", py::overload_cast<double>(&acceleration::CoM::setLambda))
            .def("setLambda", py::overload_cast<double, double>(&acceleration::CoM::setLambda))
            .def("getLambda2", &acceleration::CoM::getLambda2)
            .def("getCachedVelocityReference", &acceleration::CoM::getCachedVelocityReference)
            .def("getCachedAccelerationReference", &acceleration::CoM::getCachedAccelerationReference)
            .def("getKp", &acceleration::CoM::getKp)
            .def("getKd", &acceleration::CoM::getKd)
            .def("setKp", &acceleration::CoM::setKp)
            .def("setKd", &acceleration::CoM::setKd)
            .def("setGains", &acceleration::CoM::setGains);
}

void pyDynamicFeasibility(py::module& m) {
    py::class_<acceleration::DynamicFeasibility, std::shared_ptr<acceleration::DynamicFeasibility>, OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>>(m, "DynamicFeasibility")
        .def(py::init<const std::string&, const XBot::ModelInterface&, const AffineHelper&,
                      const std::vector<AffineHelper>&, const std::vector<std::string>&>())
        .def("enableContact", &acceleration::DynamicFeasibility::enableContact)
        .def("disableContact", &acceleration::DynamicFeasibility::disableContact)
        .def("getEnabledContacts", &acceleration::DynamicFeasibility::getEnabledContacts)
        .def("checkTask", &acceleration::DynamicFeasibility::checkTask);
}
