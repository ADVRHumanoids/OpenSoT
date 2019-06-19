#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

const std::string OpenSoT::tasks::acceleration::Cartesian::world_name = "world";

OpenSoT::tasks::acceleration::Cartesian::Cartesian(const std::string task_id,
          const Eigen::VectorXd& x,
          const XBot::ModelInterface& robot,
          const std::string& distal_link,
          const std::string& base_link
         ):
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, x.size()),
    _robot(robot),
    _distal_link(distal_link),
    _base_link(base_link),
    _orientation_gain(1.0)
{
    _qddot = AffineHelper::Identity(x.size());

    resetReference();

    _hessianType = HST_SEMIDEF;

    _vel_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref.setZero();
    _acc_ref_cached = _acc_ref;

    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);

    _Kp.setIdentity();
    _Kd.setIdentity();
    
    update(Eigen::VectorXd(1));

    setWeight(Eigen::MatrixXd::Identity(6,6));

    
}

OpenSoT::tasks::acceleration::Cartesian::Cartesian(const std::string task_id,
                                                   const XBot::ModelInterface& robot, 
                                                   const std::string& distal_link, 
                                                   const std::string& base_link, 
                                                   const OpenSoT::AffineHelper& qddot): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _distal_link(distal_link),
    _base_link(base_link),
    _qddot(qddot),
    _orientation_gain(1.0)
{
    resetReference();

    _hessianType = HST_SEMIDEF;
    
    _vel_ref.setZero();
    _vel_ref_cached = _vel_ref;
    _acc_ref.setZero();
    _acc_ref_cached = _acc_ref;
    
    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);
    
    update(Eigen::VectorXd(1));
    
    setWeight(Eigen::MatrixXd::Identity(6,6));
    
}

const std::string& OpenSoT::tasks::acceleration::Cartesian::getBaseLink() const
{
    return _base_link;
}

const std::string& OpenSoT::tasks::acceleration::Cartesian::getDistalLink() const
{
    return _distal_link;
}

void OpenSoT::tasks::acceleration::Cartesian::setOrientationGain(double orientation_gain)
{
    if(orientation_gain < 0)
    {
        Logger::error("in %s: orientation gain is %f < 0! \n", __func__, orientation_gain);
        return;
    }
    
    _orientation_gain = orientation_gain;
}

const double OpenSoT::tasks::acceleration::Cartesian::getOrientationErrorGain() const
{
    return _orientation_gain;
}


void OpenSoT::tasks::acceleration::Cartesian::_update(const Eigen::VectorXd& x)
{
    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    if(_base_link == world_name){
        _robot.getJacobian(_distal_link, _J);
        _robot.getPose(_distal_link, _pose_current);
        _robot.getVelocityTwist(_distal_link, _vel_current);
        _robot.computeJdotQdot(_distal_link, Eigen::Vector3d::Zero(), _jdotqdot);
    }
    else{
        _robot.getRelativeJacobian(_distal_link, _base_link, _J);
        _robot.getPose(_distal_link, _base_link, _pose_current);
        _robot.getVelocityTwist(_distal_link, _base_link, _vel_current);
        _robot.computeRelativeJdotQdot(_distal_link, _base_link, _jdotqdot);
        //_jdotqdot.setZero(); ///TODO: computeJdotQdot relative!
    }
    
    XBot::Utils::computeOrientationError(_pose_ref.linear(), _pose_current.linear(), _orientation_error);
    
    _pose_error.head<3>() = _pose_ref.translation() - _pose_current.translation();
    _pose_error.tail<3>() = _orientation_gain * _orientation_error;
    
    _velocity_error = _vel_ref - _vel_current; ///Maybe here we should multiply the _orientation_gain as well?

    _cartesian_task = _J*_qddot + _jdotqdot;
    _cartesian_task = _cartesian_task - _acc_ref 
                                      - _lambda2*_Kd*_velocity_error
                                      - _lambda*_Kp*_pose_error ;
    
    _A = _cartesian_task.getM();
    _b = -_cartesian_task.getq();
    
    _vel_ref.setZero();
    _acc_ref.setZero();
}

void OpenSoT::tasks::acceleration::Cartesian::setPositionReference(const Eigen::Vector3d& pos_ref)
{
    _pose_ref.translation() = pos_ref;
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const KDL::Frame& ref)
{
    tf::transformKDLToEigen(ref, _pose_ref);
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& ref)
{
    _pose_ref = ref;
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                                                           const Eigen::Vector6d& vel_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const KDL::Frame& pose_ref,
                                                            const KDL::Twist& vel_ref)
{
    tf::transformKDLToEigen(pose_ref, _pose_ref);
    tf::twistKDLToEigen(vel_ref, _vel_ref);
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                                                           const Eigen::Vector6d& vel_ref,
                                                           const Eigen::Vector6d& acc_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref = acc_ref;

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const KDL::Frame& pose_ref,
                  const KDL::Twist& vel_ref,
                  const KDL::Twist& acc_ref)
{
    tf::transformKDLToEigen(pose_ref, _pose_ref);
    tf::twistKDLToEigen(vel_ref, _vel_ref);
    tf::twistKDLToEigen(acc_ref, _acc_ref);

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::setLambda(double lambda)
{
    if(lambda < 0){
        XBot::Logger::error("in %s: illegal lambda (%f < 0) \n", __func__, lambda);
        return;
    }
    _lambda = lambda;
    _lambda2 = 2*std::sqrt(lambda);
}

void OpenSoT::tasks::acceleration::Cartesian::setLambda(double lambda1, double lambda2)
{
    if( lambda1 < 0 || lambda2 < 0 )
    {
        XBot::Logger::error("in %s: illegal lambda (%f < 0 || %f < 0) \n", __func__, lambda1, lambda2);
        return;
    }
    
    _lambda = lambda1;
    _lambda2 = lambda2;
}

void OpenSoT::tasks::acceleration::Cartesian::resetReference()
{
    if(_base_link == world_name){
        _robot.getPose(_distal_link, _pose_ref);
    }
    else{
        _robot.getPose(_distal_link, _base_link, _pose_ref);
    }
}

bool OpenSoT::tasks::acceleration::Cartesian::reset()
{
    resetReference();
    _vel_ref.setZero();
    _acc_ref.setZero();

    _vel_ref_cached = _vel_ref;
    _acc_ref_cached = _acc_ref;

    return true;
}


void OpenSoT::tasks::acceleration::Cartesian::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(getTaskID() + "_pose_error", _pose_error);
    logger->add(getTaskID() + "_velocity_error", _velocity_error);
    logger->add(getTaskID() + "_jdotqdot", _jdotqdot);

    logger->add(getTaskID() + "_velocity_reference", _vel_ref_cached);
    logger->add(getTaskID() + "_acceleration_reference", _acc_ref_cached);
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(Eigen::Affine3d& ref)
{
    ref = _pose_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(KDL::Frame& ref)
{
    tf::transformEigenToKDL(_pose_ref, ref);
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(Eigen::Affine3d& desiredPose,
                  Eigen::Vector6d& desiredTwist)
{
    desiredPose = _pose_ref;
    desiredTwist = _vel_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(KDL::Frame& desiredPose,
                  KDL::Twist& desiredTwist)
{
    tf::transformEigenToKDL(_pose_ref, desiredPose);
    tf::twistEigenToKDL(_vel_ref, desiredTwist);
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(Eigen::Affine3d& desiredPose,
                  Eigen::Vector6d& desiredTwist,
                  Eigen::Vector6d& desiredAcceleration)
{
    desiredPose = _pose_ref;
    desiredTwist = _vel_ref;
    desiredAcceleration = _acc_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(KDL::Frame& desiredPose,
                  KDL::Twist& desiredTwist,
                  KDL::Twist& desiredAcceleration)
{
    tf::transformEigenToKDL(_pose_ref, desiredPose);
    tf::twistEigenToKDL(_vel_ref, desiredTwist);
    tf::twistEigenToKDL(_acc_ref, desiredAcceleration);
}


void OpenSoT::tasks::acceleration::Cartesian::getActualPose(Eigen::Affine3d& actual)
{
    actual = _pose_current;
}

void OpenSoT::tasks::acceleration::Cartesian::getActualPose(KDL::Frame& actual)
{
    tf::transformEigenToKDL(_pose_current, actual);
}

void OpenSoT::tasks::acceleration::Cartesian::getActualTwist(Eigen::Vector6d& actual)
{
    actual = _vel_current;
}

void OpenSoT::tasks::acceleration::Cartesian::getActualTwist(KDL::Twist& actual)
{
    tf::twistEigenToKDL(_vel_current, actual);
}

const bool OpenSoT::tasks::acceleration::Cartesian::baseLinkIsWorld() const
{
    return _base_link == world_name;
}

bool OpenSoT::tasks::acceleration::Cartesian::isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::acceleration::Cartesian>(task);
}

OpenSoT::tasks::acceleration::Cartesian::Ptr OpenSoT::tasks::acceleration::Cartesian::asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<OpenSoT::tasks::acceleration::Cartesian>(task);
}

bool OpenSoT::tasks::acceleration::Cartesian::setDistalLink(const std::string& distal_link)
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

bool OpenSoT::tasks::acceleration::Cartesian::setBaseLink(const std::string& base_link)
{
    if(base_link.compare(_base_link) == 0)
        return true;

    if(base_link.compare("world") == 0)
        _robot.getPose(_base_link, _tmpMatrix);
    else if(_base_link.compare("world") == 0){
        _robot.getPose(base_link, _tmpMatrix2);
        _tmpMatrix = _tmpMatrix2.inverse();
    }
    else if(_robot.getLinkID(base_link) == -1)
        return false;
    else
        _robot.getPose(_base_link, base_link, _tmpMatrix);

    _base_link = base_link;
    _tmpMatrix2 = _tmpMatrix*_pose_ref;
    _pose_ref = _tmpMatrix2;

    return true;
}

const Eigen::Vector6d OpenSoT::tasks::acceleration::Cartesian::getError() const
{
    return _pose_error;
}

const Eigen::Vector6d OpenSoT::tasks::acceleration::Cartesian::getVelocityError() const
{
    return _velocity_error;
}

void OpenSoT::tasks::acceleration::Cartesian::getLambda(double & lambda, double & lambda2)
{
    lambda = _lambda;
    lambda2 = _lambda2;
}

const double OpenSoT::tasks::acceleration::Cartesian::getLambda2() const
{
    return _lambda2;
}

const Eigen::Vector6d& OpenSoT::tasks::acceleration::Cartesian::getCachedVelocityReference() const
{
    return _vel_ref_cached;
}

const Eigen::Vector6d& OpenSoT::tasks::acceleration::Cartesian::getCachedAccelerationReference() const
{
    return _acc_ref_cached;
}

void OpenSoT::tasks::acceleration::Cartesian::setKp(const Eigen::Matrix6d& Kp)
{
    _Kp = Kp;
}

void OpenSoT::tasks::acceleration::Cartesian::setKd(const Eigen::Matrix6d& Kd)
{
    _Kd = Kd;
}

void OpenSoT::tasks::acceleration::Cartesian::setGains(const Eigen::Matrix6d& Kp, const Eigen::Matrix6d& Kd)
{
    setKp(Kp);
    setKd(Kd);
}

const Eigen::Matrix6d& OpenSoT::tasks::acceleration::Cartesian::getKp() const
{
    return _Kp;
}

const Eigen::Matrix6d& OpenSoT::tasks::acceleration::Cartesian::getKd() const
{
    return _Kd;
}

void OpenSoT::tasks::acceleration::Cartesian::getGains(Eigen::Matrix6d& Kp, Eigen::Matrix6d& Kd)
{
    Kp = _Kp;
    Kd = _Kd;
}
