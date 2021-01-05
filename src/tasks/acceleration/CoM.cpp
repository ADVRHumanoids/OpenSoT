#include <OpenSoT/tasks/acceleration/CoM.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

const std::string OpenSoT::tasks::acceleration::CoM::world_name = "world";

OpenSoT::tasks::acceleration::CoM::CoM(const XBot::ModelInterface& robot, const Eigen::VectorXd& x):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("CoM", x.size()),
    _robot(robot),
    _distal_link("CoM"),
    _base_link(world_name)
{
    _qddot = AffineHelper::Identity(x.size());

    _hessianType = HST_SEMIDEF;

    resetReference();

    _vel_ref.setZero();
    _acc_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);

    update(Eigen::VectorXd(1));

    setWeight(Eigen::MatrixXd::Identity(3,3));
}

OpenSoT::tasks::acceleration::CoM::CoM(const XBot::ModelInterface &robot, const AffineHelper &qddot):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("CoM", qddot.getInputSize()),
    _robot(robot),
    _distal_link("CoM"),
    _base_link(world_name),
    _qddot(qddot)
{
    resetReference();

    _hessianType = HST_SEMIDEF;

    _vel_ref.setZero();
    _acc_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);

    update(Eigen::VectorXd(1));

    setWeight(Eigen::MatrixXd::Identity(3,3));
}

const std::string& OpenSoT::tasks::acceleration::CoM::getBaseLink() const
{
    return _base_link;
}

const std::string& OpenSoT::tasks::acceleration::CoM::getDistalLink() const
{
    return _distal_link;
}

void OpenSoT::tasks::acceleration::CoM::_update(const Eigen::VectorXd& x)
{
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    _robot.getCOMJacobian(_J, _jdotqdot);
    _robot.getCOM(_pose_current);
    _robot.getCOMVelocity(_vel_current);


    _pose_error = _pose_ref - _pose_current;

    _cartesian_task = _J*_qddot + _jdotqdot;
    _cartesian_task = _cartesian_task - _acc_ref
                                      - _lambda2*(_vel_ref - _vel_current)
                                      - _lambda*_pose_error ;

    _A = _cartesian_task.getM();
    _b = -_cartesian_task.getq();

    _vel_ref.setZero();
    _acc_ref.setZero();
}

void OpenSoT::tasks::acceleration::CoM::setReference(const Eigen::Vector3d& ref)
{
    _pose_ref = ref;
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::CoM::setReference(const Eigen::Vector3d& pose_ref,
                                                           const Eigen::Vector3d& vel_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::CoM::setReference(const Eigen::Vector3d& pose_ref,
                                                           const Eigen::Vector3d& vel_ref,
                                                           const Eigen::Vector3d& acc_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref = acc_ref;

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::CoM::setLambda(double lambda)
{
    if(lambda < 0){
        XBot::Logger::error("in %s: illegal lambda (%f < 0) \n", __func__, lambda);
        return;
    }
    _lambda = lambda;
    _lambda2 = 2*std::sqrt(lambda);
}

void OpenSoT::tasks::acceleration::CoM::setLambda(double lambda1, double lambda2)
{
    if( lambda1 < 0 || lambda2 < 0 )
    {
        XBot::Logger::error("in %s: illegal lambda (%f < 0 || %f < 0) \n", __func__, lambda1, lambda2);
        return;
    }

    _lambda = lambda1;
    _lambda2 = lambda2;
}

void OpenSoT::tasks::acceleration::CoM::resetReference()
{
    _robot.getCOM(_pose_ref);
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::CoM::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add(getTaskID() + "_pose_error", _pose_error);
    logger->add(getTaskID() + "_linear_velocity_error", _vel_ref - _vel_current);
    logger->add(getTaskID() + "_jdotqdot", _jdotqdot);

    logger->add(getTaskID() + "_velocity_reference", _vel_ref_cached);
    logger->add(getTaskID() + "_acceleration_reference", _acc_ref_cached);
}

void OpenSoT::tasks::acceleration::CoM::getReference(Eigen::Vector3d& ref)
{
    ref = _pose_ref;
}

void OpenSoT::tasks::acceleration::CoM::getActualPose(Eigen::Vector3d& actual)
{
    actual = _pose_current;
}

void OpenSoT::tasks::acceleration::CoM::getPosError(Eigen::Vector3d& error)
{
    error = _pose_error;
}

void OpenSoT::tasks::acceleration::CoM::getLambda(double & lambda, double & lambda2)
{
    lambda = _lambda;
    lambda2 = _lambda2;
}

const double OpenSoT::tasks::acceleration::CoM::getLambda2() const
{
    return _lambda2;
}

const Eigen::Vector3d& OpenSoT::tasks::acceleration::CoM::getCachedVelocityReference() const
{
    return _vel_ref_cached;
}

const Eigen::Vector3d& OpenSoT::tasks::acceleration::CoM::getCachedAccelerationReference() const
{
    return _acc_ref_cached;
}
