#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <xbot2_interface/logger.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

using XBot::Logger;
using namespace OpenSoT::tasks::acceleration;

const std::string Cartesian::world_name = "world";

Cartesian::Cartesian(const std::string task_id,
                     const XBot::ModelInterface& robot,
                     const std::string& distal_link,
                     const std::string& base_link
                     ):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, robot.getNv()),
    _robot(robot),
    _distal_link(distal_link),
    _base_link(base_link),
    _orientation_gain(1.0),
    _gain_type(GainType::Acceleration)
{
    _qddot = AffineHelper::Identity(_x_size);

    resetReference();

    _hessianType = HST_SEMIDEF;

    _vel_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref.setZero();
    _acc_ref_cached = _acc_ref;

    _virtual_force_ref.setZero();
    _virtual_force_ref_cached = _virtual_force_ref;
    _tmpMatrixXd.resize(6,robot.getJointNum());
    _Bi.resize(robot.getJointNum(),robot.getJointNum());

    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);

    _Kp.setIdentity();
    _Kd.setIdentity();
    
    update();

    setWeight(Eigen::MatrixXd::Identity(6,6));

    
}

Cartesian::Cartesian(const std::string task_id,
                     const XBot::ModelInterface& robot,
                     const std::string& distal_link,
                     const std::string& base_link,
                     const OpenSoT::AffineHelper& qddot):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _distal_link(distal_link),
    _base_link(base_link),
    _qddot(qddot),
    _orientation_gain(1.0),
    _gain_type(GainType::Acceleration)
{
    resetReference();

    _hessianType = HST_SEMIDEF;
    
    _vel_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref.setZero();
    _acc_ref_cached = _acc_ref;

    _virtual_force_ref.setZero();
    _virtual_force_ref_cached = _virtual_force_ref;
    _tmpMatrixXd.resize(6,robot.getJointNum());
    _Bi.resize(robot.getJointNum(),robot.getJointNum());
    
    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);

    _Kp.setIdentity();
    _Kd.setIdentity();
    
    update();
    
    setWeight(Eigen::MatrixXd::Identity(6,6));
    
}

void Cartesian::setGainType(GainType type)
{
    _gain_type = type;
}

OpenSoT::tasks::acceleration::GainType Cartesian::getGainType() const
{
    return _gain_type;
}

const std::string& Cartesian::getBaseLink() const
{
    return _base_link;
}

const std::string& Cartesian::getDistalLink() const
{
    return _distal_link;
}

void Cartesian::setOrientationGain(double orientation_gain)
{
    if(orientation_gain < 0)
    {
        Logger::error("in %s: orientation gain is %f < 0! \n", __func__, orientation_gain);
        return;
    }
    
    _orientation_gain = orientation_gain;
}

const double Cartesian::getOrientationErrorGain() const
{
    return _orientation_gain;
}


void Cartesian::_update()
{
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
    _virtual_force_ref_cached = _virtual_force_ref;

    if(_base_link == world_name){
        _robot.getJacobian(_distal_link, _J);
        _robot.getPose(_distal_link, _pose_current);
        _robot.getVelocityTwist(_distal_link, _vel_current);
        _robot.getJdotTimesV(_distal_link, _jdotqdot);
    }
    else{
        _robot.getRelativeJacobian(_distal_link, _base_link, _J);
        _robot.getPose(_distal_link, _base_link, _pose_current);
        _vel_current = _robot.getRelativeVelocityTwist(_distal_link, _base_link);
        _robot.getRelativeJdotTimesV(_distal_link, _base_link, _jdotqdot);
        //_jdotqdot.setZero(); ///TODO: computeJdotQdot relative!
    }
    
    XBot::Utils::computeOrientationError(_pose_ref.linear(), _pose_current.linear(), _orientation_error);
    
    _pose_error.head<3>() = _pose_ref.translation() - _pose_current.translation();
    _pose_error.tail<3>() = _orientation_gain * _orientation_error;
    
    _velocity_error = _vel_ref - _vel_current; ///Maybe here we should multiply the _orientation_gain as well?

    _cartesian_task = _J*_qddot + _jdotqdot;
    if(_gain_type == Acceleration)
    {
        _cartesian_task = _cartesian_task - _acc_ref
                          - _lambda2*_Kd*_velocity_error
                          - _lambda*_Kp*_pose_error;
    }
    else if(_gain_type == Force)
    {
        compute_cartesian_inertia_inverse();

        _cartesian_task = _cartesian_task - _acc_ref
                          - _lambda2*_Mi*_Kd*_velocity_error
                          - _lambda*_Mi*_Kp*_pose_error
                          - _Mi*_virtual_force_ref;
    }
    else
    {
        // we should never get here
        throw std::runtime_error("unsupported GainType value");
    }
    
    _A = _cartesian_task.getM();
    _b = -_cartesian_task.getq();
    
    _vel_ref.setZero();
    _acc_ref.setZero();
    _virtual_force_ref.setZero();
}

void Cartesian::setPositionReference(const Eigen::Vector3d& pos_ref)
{
    _pose_ref.translation() = pos_ref;
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setReference(const Eigen::Affine3d& ref)
{
    _pose_ref = ref;
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                             const Eigen::Vector6d& vel_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setReference(const KDL::Frame& pose_ref,
                             const KDL::Twist& vel_ref)
{
    tf2::transformKDLToEigen(pose_ref, _pose_ref);
    tf2::twistKDLToEigen(vel_ref, _vel_ref);
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                             const Eigen::Vector6d& vel_ref,
                             const Eigen::Vector6d& acc_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref = acc_ref;

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setReference(const KDL::Frame& pose_ref,
                             const KDL::Twist& vel_ref,
                             const KDL::Twist& acc_ref)
{
    tf2::transformKDLToEigen(pose_ref, _pose_ref);
    tf2::twistKDLToEigen(vel_ref, _vel_ref);
    tf2::twistKDLToEigen(acc_ref, _acc_ref);

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void Cartesian::setVirtualForce(const Eigen::Vector6d& virtual_force_ref)
{
    _virtual_force_ref = virtual_force_ref;
}

void Cartesian::setLambda(double lambda)
{
    if(lambda < 0){
        XBot::Logger::error("in %s: illegal lambda (%f < 0) \n", __func__, lambda);
        return;
    }
    _lambda = lambda;
    _lambda2 = 2*std::sqrt(lambda);
}

void Cartesian::setLambda(double lambda1, double lambda2)
{
    if( lambda1 < 0 || lambda2 < 0 )
    {
        XBot::Logger::error("in %s: illegal lambda (%f < 0 || %f < 0) \n", __func__, lambda1, lambda2);
        return;
    }
    
    _lambda = lambda1;
    _lambda2 = lambda2;
}

void Cartesian::resetReference()
{
    if(_base_link == world_name){
        _robot.getPose(_distal_link, _pose_ref);
    }
    else{
        _robot.getPose(_distal_link, _base_link, _pose_ref);
    }
}

bool Cartesian::reset()
{
    resetReference();
    _vel_ref.setZero();
    _acc_ref.setZero();
    _virtual_force_ref.setZero();

    _virtual_force_ref_cached = _virtual_force_ref;
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    return true;
}


void Cartesian::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add(getTaskID() + "_pose_error", _pose_error);
    logger->add(getTaskID() + "_velocity_error", _velocity_error);
    logger->add(getTaskID() + "_jdotqdot", _jdotqdot);

    logger->add(getTaskID() + "_velocity_reference", _vel_ref_cached);
    logger->add(getTaskID() + "_acceleration_reference", _acc_ref_cached);
    logger->add(getTaskID() +  "_virtual_force_reference", _virtual_force_ref_cached);

    logger->add(getTaskID() + "_lambda2", _lambda2);
}

void Cartesian::getReference(Eigen::Affine3d& ref) const
{
    ref = _pose_ref;
}

void Cartesian::getReference(KDL::Frame& ref) const
{
    tf2::transformEigenToKDL(_pose_ref, ref);
}

void Cartesian::getReference(Eigen::Affine3d& desiredPose,
                             Eigen::Vector6d& desiredTwist) const
{
    desiredPose = _pose_ref;
    desiredTwist = _vel_ref;
}

void Cartesian::getReference(KDL::Frame& desiredPose,
                             KDL::Twist& desiredTwist) const
{
    tf2::transformEigenToKDL(_pose_ref, desiredPose);
    tf2::twistEigenToKDL(_vel_ref, desiredTwist);
}

void Cartesian::getReference(Eigen::Affine3d& desiredPose,
                             Eigen::Vector6d& desiredTwist,
                             Eigen::Vector6d& desiredAcceleration) const
{
    desiredPose = _pose_ref;
    desiredTwist = _vel_ref;
    desiredAcceleration = _acc_ref;
}

void Cartesian::getReference(KDL::Frame& desiredPose,
                             KDL::Twist& desiredTwist,
                             KDL::Twist& desiredAcceleration) const
{
    tf2::transformEigenToKDL(_pose_ref, desiredPose);
    tf2::twistEigenToKDL(_vel_ref, desiredTwist);
    tf2::twistEigenToKDL(_acc_ref, desiredAcceleration);
}


void Cartesian::getActualPose(Eigen::Affine3d& actual) const
{
    actual = _pose_current;
}

void Cartesian::getActualPose(KDL::Frame& actual)
{
    tf2::transformEigenToKDL(_pose_current, actual);
}

const Eigen::Affine3d& Cartesian::getActualPose() const
{
    return _pose_current;
}

void Cartesian::getActualTwist(Eigen::Vector6d& actual) const
{
    actual = _vel_current;
}

void Cartesian::getActualTwist(KDL::Twist& actual)
{
    tf2::twistEigenToKDL(_vel_current, actual);
}

const Eigen::Vector6d& Cartesian::getActualTwist() const
{
    return _vel_current;
}

const bool Cartesian::baseLinkIsWorld() const
{
    return _base_link == world_name;
}

bool Cartesian::isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)std::dynamic_pointer_cast<Cartesian>(task);
}

Cartesian::Ptr Cartesian::asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return std::dynamic_pointer_cast<Cartesian>(task);
}

bool Cartesian::setDistalLink(const std::string& distal_link)
{
    if(distal_link.compare(_distal_link) == 0){
        return true;
    }

    if(distal_link.compare("world") == 0){
        std::cerr << "Error in " << __func__ << ": cannot pass world as distal link." << std::endl;
        return false;
    }

    if(!_robot.getPose(distal_link, _base_link, _base_T_distal)){
        std::cerr << "Error in " << __func__ << ": base link -> distal link transform cannot be obtained." << std::endl;
        return false;
    }

    _distal_link = distal_link;

    setReference(_base_T_distal);

    return true;
}

bool Cartesian::setBaseLink(const std::string& base_link)
{
    if(base_link.compare(_base_link) == 0)
        return true;

    if(base_link.compare("world") == 0)
        _robot.getPose(_base_link, _tmpMatrix);
    else if(_base_link.compare("world") == 0){
        _robot.getPose(base_link, _tmpMatrix2);
        _tmpMatrix = _tmpMatrix2.inverse();
    }
    else if(_robot.getLinkId(base_link) < 0)
        return false;
    else
        _robot.getPose(_base_link, base_link, _tmpMatrix);

    _base_link = base_link;
    _tmpMatrix2 = _tmpMatrix*_pose_ref;
    _pose_ref = _tmpMatrix2;

    return true;
}

const Eigen::Vector6d& Cartesian::getError() const
{
    return _pose_error;
}

const Eigen::Vector6d& Cartesian::getVelocityError() const
{
    return _velocity_error;
}

void Cartesian::getLambda(double & lambda, double & lambda2)
{
    lambda = _lambda;
    lambda2 = _lambda2;
}

const double Cartesian::getLambda2() const
{
    return _lambda2;
}

const Eigen::Vector6d& Cartesian::getCachedVelocityReference() const
{
    return _vel_ref_cached;
}

const Eigen::Vector6d& Cartesian::getCachedAccelerationReference() const
{
    return _acc_ref_cached;
}

const Eigen::Vector6d& Cartesian::getCachedVirtualForceReference() const
{
    return _virtual_force_ref_cached;
}

void Cartesian::setKp(const Eigen::Matrix6d& Kp)
{
    _Kp = Kp;
}

void Cartesian::setKd(const Eigen::Matrix6d& Kd)
{
    _Kd = Kd;
}

void Cartesian::setGains(const Eigen::Matrix6d& Kp, const Eigen::Matrix6d& Kd)
{
    setKp(Kp);
    setKd(Kd);
}

const Eigen::Matrix6d& Cartesian::getKp() const
{
    return _Kp;
}

const Eigen::Matrix6d& Cartesian::getKd() const
{
    return _Kd;
}

void Cartesian::getGains(Eigen::Matrix6d& Kp, Eigen::Matrix6d& Kd)
{
    Kp = _Kp;
    Kd = _Kd;
}

void Cartesian::compute_cartesian_inertia_inverse()
{
    _robot.computeInertiaInverse(_Bi);

    _tmpMatrixXd.noalias() = _J*_Bi;

    _Mi.noalias() = _tmpMatrixXd*_J.transpose();
}

