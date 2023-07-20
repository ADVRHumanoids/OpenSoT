#include <OpenSoT/tasks/velocity/RigidRotation.h>
#include <XBotInterface/Utils.h>


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
    
    auto& zero_yaw_model = *XBot::ModelInterface::getModel(_model.getPathToConfig());
    Eigen::VectorXd q_zy;
    zero_yaw_model.getRobotState("home_ank_yaw_0", q_zy);
    zero_yaw_model.setJointPosition(q_zy);
    zero_yaw_model.update();
    
    _lambda = .1;
    _W.setIdentity(3,3);
    
    Eigen::Affine3d world_T_wheel;
    Eigen::Affine3d world_T_waist;
    Eigen::Affine3d waist_T_wheel;
    
    zero_yaw_model.getPose(_wheel_link_name, world_T_wheel);
    zero_yaw_model.getPose(_waist_link_name, world_T_waist);
    waist_T_wheel = world_T_waist.inverse()*world_T_wheel;
    
    Eigen::Vector3d cart_wheel_pos = world_T_wheel.translation() - world_T_waist.translation();
    
    setReference(0.0, Eigen::Vector3d(0.1,0.0,0.0), cart_wheel_pos.head<2>());
    
    Eigen::VectorXd q;
    model.getJointPosition(q);
    _update(q);
    
    _parent_parent_link = _model.getUrdf().getLink(_model.getUrdf().getLink(wheel_link_name)->parent_joint->parent_link_name)->parent_joint->parent_link_name;
    std::string _steering_joint = _model.getUrdf().getLink(_model.getUrdf().getLink(wheel_link_name)->parent_joint->parent_link_name)->parent_joint->name;
    
    std::cout << "pp link: " << _parent_parent_link << std::endl;
    std::cout << "steering joint: " << _steering_joint << std::endl;
    
    Eigen::Matrix3d world_R_pp;
    zero_yaw_model.getOrientation(_parent_parent_link, world_R_pp);
    Eigen::Matrix3d pp_R_wheel;
    pp_R_wheel = world_R_pp.transpose() * world_T_wheel.linear();
    
    _preferred_forward_axis = (pp_R_wheel*_wheel_spinning_axis).cross(world_R_pp.transpose()*_world_contact_plane_normal);
    
    std::cout << "preferred_forward_axis" << _preferred_forward_axis.transpose() << std::endl;
    
}

void OpenSoT::tasks::velocity::RigidRotation::setReference(double theta, 
                                                           const Eigen::Vector3d& cart_vref)
{
    if(cart_vref.norm() < 1e-9){
       return;
    }
    
    _cart_vref = cart_vref;
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
    Eigen::Matrix3d pp_R_cart;
    
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
    _model.getOrientation(_parent_parent_link, pp_R_cart); // world_R_pp
    pp_R_cart = pp_R_cart.transpose() * _world_R_cart;

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
    
    /* Relative vel ref is used to shape the support polygon (NOT USED) */
    _wheel_relative_vel_ref.head<2>() = _lambda * .0 * (_cart_wheel_pos_ref - _cart_wheel_pos).head<2>() / _dt; // NOTE ignore relative motion
    
    /* Wheel velocity reference (cart frame) */
    _wheel_vel_ref.head<2>() = _cart_vref.head<2>();
    _wheel_vel_ref += Eigen::Vector3d::UnitZ().cross(_cart_wheel_pos)*_cart_vref.z();
    _wheel_vel_ref += _wheel_relative_vel_ref;
    
    /* The desired forward axis is parallel to the reference velocity w.r.t. world */
    _wheel_forward_axis_ref = _wheel_vel_ref;
    _wheel_forward_axis_ref /= _wheel_forward_axis_ref.norm();
    
    /* Consider flipping forward axis ref according to a preferred axis */
    /* Margherita HACK */
    _pp_forward_axis_ref = pp_R_cart*_wheel_forward_axis_ref;
    
    _hack_dot_product = _preferred_forward_axis.dot(_pp_forward_axis_ref);
    _hack_treshold = _hack_dot_product - (-1.0*std::cos(3.1415/180*45));
    
    if( _preferred_forward_axis.dot(_pp_forward_axis_ref)  < -1.0*std::cos(3.1415/180*45) ){
        _wheel_forward_axis_ref *= -1.0;
    }
    
    _pp_forward_axis_ref = pp_R_cart*_wheel_forward_axis_ref;
    
    /* Desired rotation relative to the cart frame about the contact plane normal */
    _cart_omega_ref = _lambda*(_wheel_forward_axis).cross(_wheel_forward_axis_ref);
    
    _S.setZero(6,6);
    _S << 1, 0, 0, 0, 0, 0, 
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, _world_contact_plane_normal.transpose();
          
    /* Relative jacobian wheel-horizonatal frame */
    _Jwaist = _S*_Jwaist;
    _Jwaist.topRows<3>() += XBot::Utils::skewSymmetricMatrix(_world_R_cart*_cart_wheel_pos).transpose() * _Jwaist.bottomRows<3>();
    _Jrel = (_Jwheel - _Jwaist);
    
    _A.setZero(3, _Jrel.cols());
    _A << _Jrel.topRows<2>(), 
          _Jrel.bottomRows<1>();
          
    _b.setZero(3);
    _b << 0, 0, _world_contact_plane_normal.transpose() * _cart_omega_ref;
    
    
}

void OpenSoT::tasks::velocity::RigidRotation::_log(XBot::MatLogger2::Ptr logger)
{
    _model.getJointVelocity(_qdot);
    
    logger->add(_task_id + "_S", _S);
    
    logger->add(_task_id + "_Jrel", _Jrel);
    logger->add(_task_id + "_Jwaist", _Jwaist);
    logger->add(_task_id + "_value", _A*_qdot - _b);
    logger->add(_task_id + "_wheel_forward_axis_ref", _wheel_forward_axis_ref);
    logger->add(_task_id + "_pp_forward_axis_ref", _pp_forward_axis_ref);
    logger->add(_task_id + "_preferred_forward_axis", _preferred_forward_axis);
    logger->add(_task_id + "_wheel_forward_axis", _wheel_forward_axis);
    logger->add(_task_id + "_wheel_vel_ref", _wheel_vel_ref);
    logger->add(_task_id + "_wheel_relative_vel_ref", _wheel_relative_vel_ref);
    logger->add(_task_id + "_cart_omega_ref", _cart_omega_ref);
    logger->add(_task_id + "_cart_vref", _cart_vref);
    logger->add(_task_id + "_cart_wheel_pos", _cart_wheel_pos);
    logger->add(_task_id + "_world_R_cart", _world_R_cart);
    
    logger->add(_task_id + "_hack_dot_product", _hack_dot_product);
    logger->add(_task_id + "_hack_treshold", _hack_treshold);
    
}

