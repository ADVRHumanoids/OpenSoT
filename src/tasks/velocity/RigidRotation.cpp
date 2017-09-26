#include <OpenSoT/tasks/velocity/RigidRotation.h>

void OpenSoT::tasks::velocity::RigidRotation::setReference(const Eigen::Vector6d& twist)
{
    _waist_ref_twist = twist;
}


OpenSoT::tasks::velocity::RigidRotation::RigidRotation(std::string wheel_link_name, 
                                                       std::string waist_link_name,
                                                   const XBot::ModelInterface& model): 
    Task<Eigen::MatrixXd, Eigen::VectorXd>("RIGID_ROTATION_" + wheel_link_name, model.getJointNum()),
    _model(model),
    _wheel_link_name(wheel_link_name),
    _waist_link_name(waist_link_name),
    _wheel_spinning_axis(0, 0, 1),
    _world_contact_plane_normal(0, 0, 1),
    _waist_icr(0, 1e16, 0),
    _thetadot(1.0),
    _waist_ref_twist(Eigen::Vector6d::Zero())
{
    Eigen::VectorXd q;
    model.getJointPosition(q);
    _lambda = 1;
    _update(q);
    
    _parent_parent_link = _model.getUrdf().getLink(_model.getUrdf().getLink(wheel_link_name)->parent_joint->parent_link_name)->parent_joint->parent_link_name;
    std::cout << "pp link: " << _parent_parent_link << std::endl;
    
}

void OpenSoT::tasks::velocity::RigidRotation::setIcr()
{
    Eigen::Vector3d waist_pos;
    _model.getPointPosition(_waist_link_name, Eigen::Vector3d::Zero(), waist_pos);
    
    _thetadot = _waist_ref_twist(5);
    _thetadot = std::fabs(_thetadot) < 1e-10 ? _thetadot + 1e-9 : _thetadot;
    
    _waist_icr = Eigen::Vector3d(-_waist_ref_twist(1), _waist_ref_twist(0), 0.0)/_thetadot;
    
}


void OpenSoT::tasks::velocity::RigidRotation::_update(const Eigen::VectorXd& x)
{

    Eigen::MatrixXd _Jwaist, _Jrel;
    
    setIcr();

    /* Get some transforms */
    Eigen::Affine3d world_T_wheel;
    Eigen::Matrix3d world_R_wheel;
    Eigen::Affine3d wheel_T_waist;
    Eigen::Matrix3d wheel_R_waist;
    
    _model.getPose(_wheel_link_name, world_T_wheel);
    world_R_wheel = world_T_wheel.linear();
    _model.getPose(_waist_link_name, _wheel_link_name, wheel_T_waist);
    wheel_R_waist = wheel_T_waist.linear();

    /* Wheel jacobian and waist jacobian */
    _model.getJacobian(_wheel_link_name, _Jwheel);
    _model.getJacobian(_waist_link_name, _Jwaist);
    
    /* Wheel formard axis (wheel frame) */
    _wheel_forward_axis = (_wheel_spinning_axis).cross(world_R_wheel.transpose()*_world_contact_plane_normal);
    _wheel_forward_axis /= _wheel_forward_axis.norm();
    
    /* Wheel origin in waist frame */
    Eigen::Vector3d waist_pos_wheel = wheel_T_waist.inverse().translation();
    
    /* The desired forward axis is perpendicular to the line joining the ICR to the wheel origin */
    _wheel_forward_axis_ref = _thetadot*(world_R_wheel.transpose()*_world_contact_plane_normal).cross(wheel_R_waist*(waist_pos_wheel - _waist_icr));
    _wheel_forward_axis_ref /= _wheel_forward_axis_ref.norm();
    
    /* Consider flipping waist forward axis ref */
    /* HACK */
    Eigen::Matrix3d wheel_R_pp;
    _model.getOrientation(_parent_parent_link, _wheel_link_name, wheel_R_pp);
    if( wheel_R_pp.col(1).dot(_wheel_forward_axis_ref) < 0 ){
        _wheel_forward_axis_ref *= -1.0;
    }
    
    /* Also compute the forward axis in waist frame for logging */
    _waist_forward_axis = wheel_R_waist.transpose()*_wheel_forward_axis;
    _waist_forward_axis_ref = wheel_R_waist.transpose()*_wheel_forward_axis_ref;
    
    /* Desired rotation aboud the contact plane normal */
    _world_omega_ref = _lambda*world_R_wheel*(_wheel_forward_axis).cross(_wheel_forward_axis_ref);
    
    /* Relative jacobian wheel-horizonatal frame */
    _Jrel = _Jwheel;
    _Jrel.topRows(3) -= _Jwaist.topRows(3);
    
    _S.setZero(1,6);
    _S << 0, 0, 0, _world_contact_plane_normal.transpose();
    
    _A = _S * _Jwheel;
    _b = _world_contact_plane_normal.transpose() * _world_omega_ref;
    _W.setIdentity(1,1);
    
}

void OpenSoT::tasks::velocity::RigidRotation::_log(XBot::MatLogger::Ptr logger)
{
    _model.getJointVelocity(_qdot);
    
    logger->add(_task_id + "_S", _S);
    logger->add(_task_id + "_value", _A*_qdot - _b);
    logger->add(_task_id + "_waist_forward_axis_ref", _waist_forward_axis_ref);
    logger->add(_task_id + "_waist_forward_axis", _waist_forward_axis);
    logger->add(_task_id + "_waist_icr", _waist_icr);
    
}

