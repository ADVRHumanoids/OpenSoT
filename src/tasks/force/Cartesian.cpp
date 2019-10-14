#include <OpenSoT/tasks/force/Cartesian.h>

using namespace OpenSoT::tasks::force;

const std::string Cartesian::world_name = "world"; 

Cartesian::Cartesian(const std::string task_id, 
		     const XBot::ModelInterface& robot, 
		     const std::string& distal_link, 
		     const std::string& base_link, 
		     const OpenSoT::AffineHelper& wrench): 
		     Task< Eigen::MatrixXd, Eigen::VectorXd >(task_id, wrench.getInputSize()),
		     _robot(robot),
		     _base_link(base_link),
		     _distal_link(distal_link),
		     _wrench(wrench)
{
    _hessianType = OpenSoT::HST_SEMIDEF;
    
    resetReference();
    
    _lambda = 1.;
    
    _Kp.setIdentity();
    _Kd.setIdentity();
    
    I.setIdentity();
    
    update(Eigen::VectorXd(1));
    
    setWeight(Eigen::MatrixXd::Identity(6,6));
}

void Cartesian::_update(const Eigen::VectorXd& x)
{
  if (_base_link == world_name)
  {
      _robot.getPose(_distal_link, _pose_current);
      _robot.getVelocityTwist(_distal_link, _vel_current);
  }
  else
  {
      _robot.getPose(_distal_link, _base_link, _pose_current);
      _robot.getVelocityTwist(_distal_link, _base_link, _vel_current);
  }
  
  XBot::Utils::computeOrientationError(_pose_ref.linear(), _pose_current.linear(), _orientation_error);
  
  _pose_error.head(3) = _pose_ref.translation() - _pose_current.translation();
  _pose_error.tail(3) = _orientation_error;
  
  _vel_error = _vel_ref - _vel_current;
    
  _virtual_force = _Kp*_pose_error + _Kd*_vel_error + _force_desired;
  
  _cartesian_task = I*_wrench;
  
  _cartesian_task = _cartesian_task - _virtual_force;
  
  _A = _cartesian_task.getM();
  _b = -_cartesian_task.getq();
  
  _vel_ref.setZero();
  _force_desired.setZero();
}

void Cartesian::setReference(const Eigen::Affine3d& pose_ref)
{
  _pose_ref = pose_ref;
  
  _vel_ref.setZero();
}

void Cartesian::setReference(const Eigen::Affine3d& pose_ref, Eigen::Vector6d& vel_ref)
{
  _pose_ref = pose_ref;
  _vel_ref = vel_ref;
}

void Cartesian::setCartesianStiffness(const Eigen::Matrix6d& Kp)
{
  _Kp = Kp;
}

void Cartesian::setCartesianDamping(const Eigen::Matrix6d& Kd)
{
  _Kd = Kd;
}

void Cartesian::getCartesianStiffness(Eigen::Matrix6d& Kp)
{
  Kp = _Kp;
}

void Cartesian::_log(XBot::MatLogger::Ptr logger)
{ 
  logger->add(getTaskID() + "_pose_current", _pose_current.matrix());
  logger->add(getTaskID() + "_pose_ref", _pose_ref.matrix());
  logger->add(getTaskID() + "_pose_error", _pose_error);
  logger->add(getTaskID() + "_vel_error", _vel_error);
  logger->add(getTaskID() + "_vel_ref", _vel_ref);
  logger->add(getTaskID() + "_vel_current", _vel_current);
  logger->add(getTaskID() + "_virtual_force", _virtual_force);
  logger->add(getTaskID() + "_Kp", _Kp);
  logger->add(getTaskID() + "_Kd", _Kd);
}

void Cartesian::setForceReference(const Eigen::Vector6d& _force_ref)
{
    _force_desired = _force_ref;
}

void Cartesian::getCartesianDamping(Eigen::Matrix6d& Kd)
{
  Kd = _Kd;
}

void Cartesian::getReference(Eigen::Affine3d& pose_des)
{
  pose_des = _pose_ref;
}

void Cartesian::getReference(Eigen::Affine3d& pose_des, Eigen::Vector6d& twist_des)
{
  pose_des = _pose_ref;
  twist_des = _vel_ref;
}

void Cartesian::resetReference()
{
  _vel_ref.setZero();
  _force_desired.setZero();
  //_acc_ref.setZero();
  
  if (_base_link == world_name) {
    _robot.getPose(_distal_link, _pose_ref);
  }
  
  else {
    _robot.getPose(_distal_link, _base_link, _pose_ref);
  }
}

