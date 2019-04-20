#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>

using namespace OpenSoT::tasks::velocity;

SimpleSteering::SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                               std::string wheel_name):
    _model(model),
    _wheel_name(wheel_name),
    _comp(0.0025, 0.01),
    _vdes(1., 0, 0)
{
    _local_R_world.setIdentity();
    
    auto spinning_axis = _model->getUrdf().getLink(wheel_name)->parent_joint->axis;
    _wheel_spinning_axis << spinning_axis.x, spinning_axis.y, spinning_axis.z;
    
    _wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;
    auto steering_axis = _model->getUrdf().getLink(_wheel_parent_name)->parent_joint->axis;
    _local_steering_axis << steering_axis.x, steering_axis.y, steering_axis.z;
    Eigen::Matrix3d w_R_wp;
    _model->getOrientation(_wheel_parent_name, w_R_wp);
    
    
    _steering_id = _model->getDofIndex(_model->getUrdf().getLink(_wheel_parent_name)->parent_joint->name);
    _steering_joint = _model->getJointByName(_model->getUrdf().getLink(_wheel_parent_name)->parent_joint->name);
    
    Eigen::VectorXd q;
    _model->getJointPosition(q);
    _prev_qdes = q[_steering_id];
    
    _model->getFloatingBaseLink(_waist_name);
}

void OpenSoT::tasks::velocity::SimpleSteering::setOutwardNormal(const Eigen::Vector3d& n)
{
    Eigen::Vector3d e1(1., 0., 0.);
    Eigen::Vector3d e2(0., 1., 0.);
    Eigen::Vector3d e3(0., 0., 1.);
    
    Eigen::Vector3d uz = n;
    
    /* Find ux as the most perpendicular direction to n */
    Eigen::Vector3d ux = e1;
    
    if(std::fabs(uz.dot(e2)) < std::fabs(uz.dot(ux)))
    {
        ux = e2;
    }
    
    if(std::fabs(uz.dot(e3)) < std::fabs(uz.dot(ux)))
    {
        ux = e3;
    }
    
    Eigen::Vector3d uy = uz.cross(ux);
    
    _local_R_world << ux, uy, uz;
    _local_R_world.transposeInPlace();
    
    
}


double SimpleSteering::wrap_angle(double x)
{
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}

double SimpleSteering::getDofIndex() const
{
    return _steering_id;
}

namespace 
{
    double dead_zone(double x, double th)
    {
        if(x > th)
        {
            return x - th;
        }
        
        if(x < -th)
        {
            return x + th;
        }
        
        return 0.0;
    }
}

double SimpleSteering::computeSteeringAngle(const Eigen::Vector3d& wheel_vel)
{
    
    /* Get orientations */
    Eigen::Affine3d w_T_wheel;
    _model->getPose(_wheel_name, w_T_wheel);
    
    Eigen::Affine3d w_T_waist;
    _model->getPose(_waist_name, w_T_waist);
    
    Eigen::Affine3d w_T_wp;
    _model->getPose(_wheel_parent_name, w_T_wp);
    
    /* Steering joint angle */
    _model->getJointPosition(_q);
    double q = _q(_steering_id);
    
    /* Current angle */
    Eigen::Vector3d world_normal = _local_R_world.row(2).transpose();
    Eigen::Vector3d wheel_forward = (w_T_wheel.linear()*_wheel_spinning_axis).cross(world_normal);
    wheel_forward.normalize(); 
    wheel_forward = _local_R_world * wheel_forward; // convert to local frame
    double theta = std::atan2(wheel_forward.y(), wheel_forward.x());
    
    /* Apply deadzone */
    Eigen::Vector3d vdes_th = _local_R_world * wheel_vel;
    
    vdes_th.x() = dead_zone(vdes_th.x(), 0.01);
    vdes_th.y() = dead_zone(vdes_th.y(), 0.01);
    
    
    if(vdes_th.head(2).norm() == 0.0) 
    {
        return _prev_qdes;
    }
    
    double des_theta_1 = std::atan2(vdes_th.y(), vdes_th.x());
    
    Eigen::Vector3d steering_axis = _local_R_world*w_T_wp.linear()*_local_steering_axis;
    double des_q_1 = q  + (des_theta_1 - theta)*steering_axis.z();
    des_q_1 = wrap_angle(des_q_1);
    
    double des_q_2 = des_q_1 + M_PI;
    des_q_2 = wrap_angle(des_q_2);
    
    double closest_q = std::fabs(des_q_1-q) < std::fabs(des_q_2-q) ? des_q_1 : des_q_2;
    
    /* Return value inside joint lims */
    if(_steering_joint->checkJointLimits(closest_q))
    {
        _prev_qdes = closest_q;
    }
    else if(_steering_joint->checkJointLimits(des_q_1))
    {
        _prev_qdes = des_q_1;
    }
    else if(_steering_joint->checkJointLimits(des_q_2))
    {
        _prev_qdes = des_q_2;
    }
    else{
        throw std::runtime_error("Unable to find steering angle");
    }

    return _prev_qdes;
}

void SimpleSteering::log(XBot::MatLogger::Ptr logger)
{
    logger->add("vdes_"+_wheel_name, _vdes);
    logger->add("vdes_norm_"+_wheel_name, double(_vdes.head<2>().norm()));
    logger->add("threshold_"+_wheel_name, _comp.getCurrentThreshold());
}



HysteresisComparator::HysteresisComparator(double th_lo, double th_hi, bool init_lo):
     _th_lo(th_lo),
     _th_hi(th_hi),
     _th_curr(init_lo ? th_lo : th_hi)
{
    if(_th_hi < _th_lo)
    {
        throw std::invalid_argument("upper threshold < lower threshold");
    }
}

bool HysteresisComparator::compare(double value)
{
    bool ret = value > _th_curr;
    _th_curr = ret ? _th_lo : _th_hi;
    return ret;
}

double HysteresisComparator::getCurrentThreshold() const
{
    return _th_curr;
}

namespace
{
    std::string get_parent(std::string link, XBot::ModelInterface::ConstPtr model)
    {
        return model->getUrdf().getLink(link)->parent_joint->parent_link_name;
    }
}

CentauroAnkleSteering::CentauroAnkleSteering(std::string wheel_name,
                                             XBot::ModelInterface::ConstPtr model, 
                                             double dt,
                                             double max_steering_speed):
    Task("centauro_steering_" + wheel_name, model->getJointNum()),
    _steering(model, wheel_name),
    _max_steering_dq(max_steering_speed*dt),
    _model(model)
{
    _A.setZero(1, model->getJointNum());
    _b.setZero(1);
    _W.setIdentity(1,1);
    
    std::string wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;
    std::string steering_joint_name = _model->getUrdf().getLink(wheel_parent_name)->parent_joint->name;
    
    _steering_joint = _model->getJointByName(steering_joint_name);
    
    _model->getJointPosition(_q);
    
    _steering_dof_idx = _model->getDofIndex(steering_joint_name);
    
    _A(0, _steering_dof_idx) = 1.0;
}

void OpenSoT::tasks::velocity::CentauroAnkleSteering::setOutwardNormal(const Eigen::Vector3d& n)
{
    _steering.setOutwardNormal(n);
}


void CentauroAnkleSteering::_update(const Eigen::VectorXd& x)
{
    // get robot state
    _model->getJointPosition(_q);
    
    // compute wheel desired velocity
    std::string ankle_name = ::get_parent(_steering.getWheelName(), _model);
    Eigen::Vector6d wheel_vel;
    _model->getVelocityTwist(_steering.getWheelName(), wheel_vel);
//     Eigen::Matrix3d w_R_ankle;
//     _model->getOrientation(ankle_name, w_R_ankle);
//     wheel_vel.head<3>() += wheel_vel.tail<3>().cross(w_R_ankle * Eigen::Vector3d(0.0, 0.0, 0.30));
    
    double q_steering = _steering.computeSteeringAngle(wheel_vel.head<3>());
    double q_current = _q(_steering_dof_idx);
    
    double dq = _lambda*(q_steering - q_current);
    dq = std::min(std::max(dq, -_max_steering_dq), _max_steering_dq);
    
    _b(0) = dq;
    
}
