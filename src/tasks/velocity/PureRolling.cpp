#include <OpenSoT/tasks/velocity/PureRolling.h>

OpenSoT::tasks::velocity::PureRolling::PureRolling(std::string wheel_link_name, 
                                                   double radius, 
                                                   const XBot::ModelInterface& model): 
    Task<Eigen::MatrixXd, Eigen::VectorXd>("PURE_ROLLING_" + wheel_link_name, model.getJointNum()),
    _model(model),
    _radius(radius),
    _wheel_link_name(wheel_link_name),
    _wheel_axis(0, 0, 1),
    _world_contact_plane_normal(0, 0, 1)
{
    Eigen::VectorXd q;
    model.getJointPosition(q);
    _update(q);
}


void OpenSoT::tasks::velocity::PureRolling::_update(const Eigen::VectorXd& x)
{

    /* Compute tranforms */
    _model.getPose(_wheel_link_name, _world_T_wheel);
    _world_R_wheel = _world_T_wheel.linear();
    
    /* Compute the contact point (wheel frame) */
    _wheel_contact_point = -1.0 * _radius * _world_R_wheel.transpose() * _world_contact_plane_normal;
    
    /* Compute the jacobian of the contact point (world frame) */
    _model.getJacobian(_wheel_link_name, _wheel_contact_point, _Jc);
    
    /* The wheel forward axis is the cross product between the wheel spinning axis and the normal to the plane */
    Eigen::Vector3d _wheel_forward_axis = (_world_R_wheel*_wheel_axis).cross(_world_contact_plane_normal);
    _wheel_forward_axis /= _wheel_forward_axis.norm();
    
    _S.setIdentity(4,6);
    _S.row(3) << 0, 0, 0, _wheel_forward_axis.transpose();
    
    _A = _S * _Jc;
    _b.setZero(4);
    _W.setIdentity(4,4);
    

    
}

void OpenSoT::tasks::velocity::PureRolling::_log(XBot::MatLogger::Ptr logger)
{
    _model.getJointVelocity(_qdot);
    
    logger->add(_task_id + "_S", _S);
    logger->add(_task_id + "_wheel_contact_point", _wheel_contact_point);
    logger->add(_task_id + "_world_contact_point", _world_T_wheel*_wheel_contact_point);
    logger->add(_task_id + "_value", _A*_qdot);
}

OpenSoT::tasks::velocity::PureRollingPosition::PureRollingPosition(std::string wheel_link_name,
                                                           double radius,
                                                           const XBot::ModelInterface& model):
    Task<Eigen::MatrixXd, Eigen::VectorXd>("PURE_ROLLING_POSITION_" + wheel_link_name, model.getJointNum())
{
    _pure_rolling.reset(new OpenSoT::tasks::velocity::PureRolling(
                            wheel_link_name, radius, model));

    _position_indices.push_back(0);
    _position_indices.push_back(1);
    _position_indices.push_back(2);

    _subtask.reset(new OpenSoT::SubTask(_pure_rolling, _position_indices));

    Eigen::VectorXd q;
    model.getJointPosition(q);
    _update(q);
}

void OpenSoT::tasks::velocity::PureRollingPosition::_update(const Eigen::VectorXd &x)
{
    _subtask->update(x);

    _A = _subtask->getA();
    _b = _subtask->getb();
}

OpenSoT::tasks::velocity::PureRollingOrientation::PureRollingOrientation(std::string wheel_link_name,
                                                           double radius,
                                                           const XBot::ModelInterface& model):
    Task<Eigen::MatrixXd, Eigen::VectorXd>("PURE_ROLLING_ORIENTATION_" + wheel_link_name, model.getJointNum())
{
    _pure_rolling.reset(new OpenSoT::tasks::velocity::PureRolling(
                            wheel_link_name, radius, model));

    _orientation_indices.push_back(3);
    _subtask.reset(new OpenSoT::SubTask(_pure_rolling, _orientation_indices));

    Eigen::VectorXd q;
    model.getJointPosition(q);
    _update(q);
}

void OpenSoT::tasks::velocity::PureRollingOrientation::_update(const Eigen::VectorXd &x)
{
    _subtask->update(x);

    _A = _subtask->getA();
    _b = _subtask->getb();
}
