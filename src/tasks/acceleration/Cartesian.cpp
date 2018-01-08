#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

const std::string OpenSoT::tasks::acceleration::Cartesian::world_name = "world";

OpenSoT::tasks::acceleration::Cartesian::Cartesian(const std::string task_id,
          const XBot::ModelInterface& robot,
          const std::string& distal_link,
          const std::string& base_link,
          const Eigen::VectorXd& x
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
    _acc_ref.setZero();

    _lambda = 100.;
    _lambda2 = 2.*sqrt(_lambda);
    
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
    _acc_ref.setZero();
    
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


void OpenSoT::tasks::acceleration::Cartesian::_update(const Eigen::VectorXd& x)
{
    if(_base_link == world_name){
        _robot.getJacobian(_distal_link, _J);
        _robot.getPose(_distal_link, _pose_current);
        _robot.getVelocityTwist(_distal_link, _vel_current);
        _robot.computeJdotQdot(_distal_link, Eigen::Vector3d::Zero(), _jdotqdot);
    }
    else{
        /* TBD implement */
    }
    
    XBot::Utils::computeOrientationError(_pose_ref.linear(), _pose_current.linear(), _orientation_error);
    
    _pose_error.head<3>() = _pose_ref.translation() - _pose_current.translation();
    _pose_error.tail<3>() = _orientation_gain * _orientation_error;
    
    _cartesian_task = _J*_qddot + _jdotqdot;
    _cartesian_task = _cartesian_task - _acc_ref 
                                      - _lambda2*(_vel_ref - _vel_current)
                                      - _lambda*_pose_error ;
    
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
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& ref)
{
    _pose_ref = ref;
    _vel_ref.setZero();
    _acc_ref.setZero();
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                                                           const Eigen::Vector6d& vel_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref.setZero();
}

void OpenSoT::tasks::acceleration::Cartesian::setReference(const Eigen::Affine3d& pose_ref,
                                                           const Eigen::Vector6d& vel_ref,
                                                           const Eigen::Vector6d& acc_ref)
{
    _pose_ref = pose_ref;
    _vel_ref = vel_ref;
    _acc_ref = acc_ref;
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
        throw std::invalid_argument("Relative cartesian task not yet supported!");
    }
}


void OpenSoT::tasks::acceleration::Cartesian::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(getTaskID() + "_pose_error", _pose_error);
    logger->add(getTaskID() + "_velocity_error", _vel_ref - _vel_current);
    logger->add(getTaskID() + "_jdotqdot", _jdotqdot);
}

void OpenSoT::tasks::acceleration::Cartesian::getReference(Eigen::Affine3d& ref)
{
    ref = _pose_ref;
}

void OpenSoT::tasks::acceleration::Cartesian::getActualPose(Eigen::Affine3d& actual)
{
    actual = _pose_current;
}
