#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>

using namespace OpenSoT::tasks::velocity;

SimpleSteering::SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                               std::string wheel_name):
    _model(model),
    _wheel_name(wheel_name),
    _comp(0.0025, 0.01)
{
    auto spinning_axis = _model->getUrdf().getLink(wheel_name)->parent_joint->axis;
    _wheel_spinning_axis << spinning_axis.x, spinning_axis.y, spinning_axis.z;
    
    std::string wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;
    auto steering_axis = _model->getUrdf().getLink(wheel_parent_name)->parent_joint->axis;
    _world_steering_axis << steering_axis.x, steering_axis.y, steering_axis.z;
    Eigen::Matrix3d w_R_wp;
    _model->getOrientation(wheel_parent_name, w_R_wp);
    _world_steering_axis = w_R_wp * _world_steering_axis;
    
    
    _steering_id = _model->getDofIndex(_model->getUrdf().getLink(wheel_parent_name)->parent_joint->name);
    _steering_joint = _model->getJointByName(_model->getUrdf().getLink(wheel_parent_name)->parent_joint->name);
    
    _model->getFloatingBaseLink(_waist_name);
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
    
    /* Wheel orientation */
    Eigen::Affine3d w_T_wheel;
    _model->getPose(_wheel_name, w_T_wheel);
    
    Eigen::Affine3d w_T_waist;
    _model->getPose(_waist_name, w_T_waist);
    
    /* Steering joint angle */
    _model->getJointPosition(_q);
    double q = _q(_steering_id);
    
    /* Current angle */
    Eigen::Vector3d wheel_forward = (w_T_wheel.linear()*_wheel_spinning_axis).cross(Eigen::Vector3d::UnitZ());
    wheel_forward.normalize();
    double theta = std::atan2(wheel_forward.y(), wheel_forward.x());
    
    /* Desired angle */
    Eigen::Vector3d r = w_T_wheel.translation() - w_T_waist.translation();
    _vdes = wheel_vel;
    
    Eigen::Vector3d vdes_th = _vdes;
    
    vdes_th.x() = dead_zone(vdes_th.x(), 0.01);
    vdes_th.y() = dead_zone(vdes_th.y(), 0.01);
    
    
    if( vdes_th.head(2).norm() == 0.0 ) 
    {
        vdes_th << 1.0, 0.0, 0.0;
        vdes_th = w_T_waist.linear() * vdes_th;
    }
    
    double des_theta_1 = std::atan2(vdes_th.y(), vdes_th.x());
    
    double des_q_1 = q  + (des_theta_1 - theta)*_world_steering_axis.z();
    des_q_1 = wrap_angle(des_q_1);
    
    double des_q_2 = des_q_1 + M_PI;
    des_q_2 = wrap_angle(des_q_2);
    
    double closest_q = std::fabs(des_q_1-q) < std::fabs(des_q_2-q) ? des_q_1 : des_q_2;
    
    /* Return value inside joint lims */
    if(_steering_joint->checkJointLimits(closest_q))
    {
        return closest_q;
    }
    else if(_steering_joint->checkJointLimits(des_q_1))
    {
        return des_q_1;
    }
    else if(_steering_joint->checkJointLimits(des_q_2))
    {
        return des_q_2;
    }
    else{
        throw std::runtime_error("Unable to find steering angle");
    }

    
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
