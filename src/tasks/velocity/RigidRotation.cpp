#include <OpenSoT/tasks/velocity/RigidRotation.h>



OpenSoT::tasks::velocity::RigidRotation::RigidRotation(std::string wheel_link_name, 
                                                       std::string waist_link_name,
                                                   const XBot::ModelInterface& model,
                                                   double dt
                                                      ): 
    Task<Eigen::MatrixXd, Eigen::VectorXd>("RIGID_ROTATION_" + wheel_link_name, model.getJointNum()),
    _model(model),
    _wheel_link_name(wheel_link_name),
    _waist_link_name(waist_link_name),
    _wheel_spinning_axis(0, 0, 1), // TBD from URDF
    _world_contact_plane_normal(0, 0, 1),
    _dt(dt)
{
    
    _lambda = .1;
    _W.setIdentity(3,3);
    
    Eigen::Affine3d world_T_wheel;
    Eigen::Affine3d world_T_waist;
    
    _model.getPose(_wheel_link_name, world_T_wheel);
    _model.getPose(_waist_link_name, world_T_waist);
    
    Eigen::Vector3d cart_wheel_pos = world_T_wheel.translation() - world_T_waist.translation();
    
    setReference(0.0, Eigen::Vector3d(0.1,0.0,0.0), cart_wheel_pos.head<2>());
    
    Eigen::VectorXd q;
    model.getJointPosition(q);
    _update(q);
    
    _parent_parent_link = _model.getUrdf().getLink(_model.getUrdf().getLink(wheel_link_name)->parent_joint->parent_link_name)->parent_joint->parent_link_name;
    std::cout << "pp link: " << _parent_parent_link << std::endl;
    
}

void OpenSoT::tasks::velocity::RigidRotation::setReference(double theta, 
                                                           const Eigen::Vector3d& cart_vref)
{
    _world_R_cart = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
    _cart_vref = cart_vref;
    if(_cart_vref.norm() < 1e-9){
        _cart_vref << 1e-9, 0, 0;
    }
}

void OpenSoT::tasks::velocity::RigidRotation::setReference(double theta, 
                                                           const Eigen::Vector3d& cart_vref, 
                                                           const Eigen::Vector2d& wheel_pos_ref)
{
    setReference(theta, cart_vref);
    _cart_wheel_pos_ref << wheel_pos_ref, 0.0;
}


void OpenSoT::tasks::velocity::RigidRotation::_update(const Eigen::VectorXd& x)
{

    /* Get some transforms */
    Eigen::Affine3d world_T_wheel;
    Eigen::Matrix3d world_R_wheel;
    Eigen::Affine3d wheel_T_waist;
    Eigen::Matrix3d wheel_R_waist;
    Eigen::Affine3d world_T_waist;
    Eigen::Matrix3d world_R_waist;
    Eigen::Matrix3d cart_R_waist;
    
    _model.getPose(_wheel_link_name, world_T_wheel);
    world_R_wheel = world_T_wheel.linear();
    _model.getPose(_waist_link_name, _wheel_link_name, wheel_T_waist);
    wheel_R_waist = wheel_T_waist.linear();
    _model.getPose(_waist_link_name, world_T_waist);
    world_R_waist = world_T_waist.linear();
    
    /* Compute world_R_cart from waist */
    double theta = std::atan2(world_R_waist(1,0), world_R_waist(0,0));
    _world_R_cart = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
    
    cart_R_waist = _world_R_cart.transpose()*world_T_waist.linear();

    /* Wheel jacobian and waist jacobian */
    _model.getJacobian(_wheel_link_name, _Jwheel);
    _model.getJacobian(_waist_link_name, _Jwaist);
    
    /* Wheel forward axis (cart frame) */
    _wheel_forward_axis = _world_R_cart.transpose() * (world_R_wheel*_wheel_spinning_axis).cross(_world_contact_plane_normal);
    _wheel_forward_axis /= _wheel_forward_axis.norm();
    
    /* Wheel origin in cart frame */
    _cart_wheel_pos = _world_R_cart.transpose()*(world_T_wheel.translation() - world_T_waist.translation());
    
    /* Reference wheel velocity (cart frame) */
    _wheel_vel_ref.setZero();
    _wheel_relative_vel_ref.setZero();
    _wheel_relative_vel_ref.head<2>() = _lambda * .0 * (_cart_wheel_pos_ref - _cart_wheel_pos).head<2>() / _dt; // NOTE ignore relative motion
    _wheel_vel_ref.head<2>() = _cart_vref.head<2>();
    _wheel_vel_ref += Eigen::Vector3d::UnitZ().cross(_cart_wheel_pos)*_cart_vref.z();
    _wheel_vel_ref += _wheel_relative_vel_ref;
    
    /* The desired forward axis is parallel to the reference velocity w.r.t. world */
    _wheel_forward_axis_ref = _wheel_vel_ref;
    _wheel_forward_axis_ref /= _wheel_forward_axis_ref.norm();
    
    /* Consider flipping waist forward axis ref */
    /* Margherita HACK */
    Eigen::Matrix3d wheel_R_pp;
    _model.getOrientation(_parent_parent_link, _wheel_link_name, wheel_R_pp);
    if( wheel_R_pp.col(1).dot(world_R_wheel.transpose()*_world_R_cart*_wheel_forward_axis_ref) < 0 ){
        _wheel_forward_axis_ref *= -1.0;
    }
    
    /* Desired rotation relative to the cart frame about the contact plane normal */
    _cart_omega_ref = _lambda*(_wheel_forward_axis).cross(_wheel_forward_axis_ref);
    
    /* Relative jacobian wheel-horizonatal frame */
    _Jrel = _Jwheel;
    _Jrel.topRows(3) -= _Jwaist.topRows(3);
    _Jrel.topRows(3) = _world_R_cart.transpose() * _Jrel.topRows(3);
    
    _S.setZero(3,6);
    _S << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
		  //0, 0, 0, 0, 0, 0,
		  0, 0, 0, _world_contact_plane_normal.transpose();
    
    _A = _S * _Jrel;
    _b.setZero(3);
    _b << (_wheel_vel_ref - _cart_vref).head<2>()*_dt, 
        _world_contact_plane_normal.transpose() * (_cart_omega_ref + Eigen::Vector3d::UnitZ()*_cart_vref.z()*_dt);
    
    
}

void OpenSoT::tasks::velocity::RigidRotation::_log(XBot::MatLogger::Ptr logger)
{
    _model.getJointVelocity(_qdot);
    
    logger->add(_task_id + "_S", _S);
    logger->add(_task_id + "_value", _A*_qdot - _b);
    logger->add(_task_id + "_wheel_forward_axis_ref", _wheel_forward_axis_ref);
    logger->add(_task_id + "_wheel_forward_axis", _wheel_forward_axis);
    logger->add(_task_id + "_wheel_vel_ref", _wheel_vel_ref);
    logger->add(_task_id + "_wheel_relative_vel_ref", _wheel_relative_vel_ref);
    logger->add(_task_id + "_cart_omega_ref", _cart_omega_ref);
    logger->add(_task_id + "_cart_vref", _cart_vref);
    logger->add(_task_id + "_cart_wheel_pos", _cart_wheel_pos);
    logger->add(_task_id + "_world_R_cart", _world_R_cart);
    
}

