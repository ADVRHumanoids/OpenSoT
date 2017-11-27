#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <XBotInterface/RtLog.hpp>
using XBot::Logger;

const std::string OpenSoT::tasks::acceleration::Cartesian::world_name = "world";

OpenSoT::tasks::acceleration::Cartesian::Cartesian(const std::string task_id,
                                                   const XBot::ModelInterface& robot, 
                                                   const std::string& distal_link, 
                                                   const std::string& base_link, 
                                                   const OpenSoT::AffineHelper& qddot): 
    Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, qddot.getInputSize()),
    _robot(robot),
    _distal_link(distal_link),
    _base_link(base_link),
    _qddot(qddot)
{
    resetReference();
    
    _vel_ref.setZero();
    _acc_ref.setZero();
    
    _lambda = 10;
    
    update(Eigen::VectorXd(1));
    
    setWeight(Eigen::MatrixXd::Identity(6,6));
    
}


void OpenSoT::tasks::acceleration::Cartesian::_update(const Eigen::VectorXd& x)
{
    if(_base_link == world_name){
        _robot.getJacobian(_distal_link, _J);
        _robot.getPose(_distal_link, _pose_current);
        _robot.getVelocityTwist(_distal_link, _vel_current);
        _robot.computeJdotQdot(_distal_link, Eigen::Vector3d::Zero(), _jdotqdot);
        
        Logger::info() << "JdQd: " << _jdotqdot.transpose() << Logger::endl();
    }
    else{
        /* TBD implement */
    }
    
    XBot::Utils::computeOrientationError(_pose_ref.linear(), _pose_current.linear(), _orientation_error);
    
    _pose_error.head<3>() = _pose_ref.translation() - _pose_current.translation();
    _pose_error.tail<3>() = _orientation_error;
    
    _cartesian_task = _J*_qddot + _jdotqdot;
    _cartesian_task = _cartesian_task - _acc_ref 
                                      - 2*_lambda*(_vel_ref - _vel_current) 
                                      - _lambda*_lambda*_pose_error ;
    
    _A = _cartesian_task.getM();
    _b = -_cartesian_task.getq();
    
}

void OpenSoT::tasks::acceleration::Cartesian::setPositionReference(const Eigen::Vector3d& pos_ref)
{
    _pose_ref.translation() = pos_ref;
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
}