#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>

using namespace OpenSoT::tasks::velocity;

SimpleSteering::SimpleSteering(XBot::ModelInterface::ConstPtr model, 
                               std::string wheel_name,
                               std::vector<double> hyst_comp,
                               std::vector<double> dz_th):
    _model(model),
    _wheel_name(wheel_name),
    _comp(HysteresisComparator::MakeHysteresisComparator()),
    _vdes(1., 0, 0),
    _dz_th(dz_th)
{
    std::memset(&_log_data, 0, sizeof(_log_data));

    if (!hyst_comp.empty())
        _comp = HysteresisComparator::MakeHysteresisComparator(hyst_comp[0], hyst_comp[1]);

    _local_R_world.setIdentity();
    
    // locate wheel spinning axis
    auto spinning_axis = _model->getUrdf().getLink(wheel_name)->parent_joint->axis;
    _wheel_spinning_axis << spinning_axis.x, spinning_axis.y, spinning_axis.z;

    // wheel parent link
    _wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;
    
    // go up the tree until the steering axis is found
    urdf::JointConstSharedPtr steering_joint;
    auto link = _model->getUrdf().getLink(_wheel_parent_name);
    while(!steering_joint && link)
    {
        auto jnt = link->parent_joint;

        if(jnt->type == urdf::Joint::REVOLUTE ||
                jnt->type == urdf::Joint::CONTINUOUS)
        {
            steering_joint = jnt;
            break;
        }

        link = _model->getUrdf().getLink(jnt->parent_link_name);
    }

    // steering joint not found
    if(!steering_joint)
    {
        throw std::runtime_error("steering joint not found for wheel " + wheel_name);
    }

//    std::cout << "found steering joint " << steering_joint->name << "\n";

    // re-define wheel parent
    _wheel_parent_name = steering_joint->parent_link_name;

    // tf from parent frame to joint origin
    Eigen::Affine3d wp_T_jo;
    wp_T_jo.translation() << steering_joint->parent_to_joint_origin_transform.position.x,
            steering_joint->parent_to_joint_origin_transform.position.y,
            steering_joint->parent_to_joint_origin_transform.position.z;
    Eigen::Quaterniond qp_q_jo(
                steering_joint->parent_to_joint_origin_transform.rotation.w,
                steering_joint->parent_to_joint_origin_transform.rotation.x,
                steering_joint->parent_to_joint_origin_transform.rotation.y,
                steering_joint->parent_to_joint_origin_transform.rotation.z
                );
    wp_T_jo.linear() = qp_q_jo.toRotationMatrix();


//    std::cout << "found wheel parent link " << _wheel_parent_name << "\n";

    // get steering axis in joint frame
    auto steering_axis = steering_joint->axis;
    _local_steering_axis << steering_axis.x, steering_axis.y, steering_axis.z;

    // rotate to parent frame
    _local_steering_axis = wp_T_jo.linear() * _local_steering_axis;

//    std::cout << "local steering axis " << _local_steering_axis.transpose() << "\n";

    // save steering index and joint
    _steering_id = _model->getDofIndex(steering_joint->name);
    _steering_joint = _model->getJointByName(steering_joint->name);
    
    // initialize previous steering angle
    Eigen::VectorXd q;
    _model->getJointPosition(q);
    _prev_qdes = q[_steering_id];
    
    // why do we need the base link name?
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
    
    double sign(double x)
    {
        return (x > 0) - (x < 0);
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
    
    /* Current angle between forward wheel direction and global x-axis */
    Eigen::Vector3d world_normal = _local_R_world.row(2).transpose();
    Eigen::Vector3d wheel_forward = (w_T_wheel.linear()*_wheel_spinning_axis).cross(world_normal);
    wheel_forward.normalize(); 
    wheel_forward = _local_R_world * wheel_forward; // convert to local frame
    double theta = std::atan2(wheel_forward.y(), wheel_forward.x());
    
    /* Apply deadzone to wheel velocity */
    Eigen::Vector3d vdes_th = _local_R_world * wheel_vel;
    
    vdes_th.x() = dead_zone(vdes_th.x(), _dz_th[0]);
    vdes_th.y() = dead_zone(vdes_th.y(), _dz_th[1]);

//    std::cout << _wheel_name << ", normal  dir =  " << _local_R_world.row(2) << std::endl;
//    std::cout << _wheel_name << ", forward dir =  " << wheel_forward.transpose() << std::endl;
//    std::cout << _wheel_name << ", theta       =  " << theta << std::endl;
//    std::cout << _wheel_name << ", vdes        =  " << vdes_th.transpose() << std::endl;
    
    _log_data.q = q;
    _log_data.normal = _local_R_world.transpose().col(2);
    _log_data.forward = wheel_forward;
    _log_data.theta = theta;
    _log_data.vdes = vdes_th;

    if(vdes_th.head(2).norm() == 0.0) 
    {
        return _prev_qdes;
    }
    
    // desired angle based on time-delayed velocity
    double des_theta_1 = std::atan2(vdes_th.y(), vdes_th.x());
    
    Eigen::Vector3d steering_axis = _local_R_world*w_T_wp.linear()*_local_steering_axis;
    
//    std::cout << _wheel_name << ", theta ref   = " << des_theta_1 << std::endl;
//    std::cout << _wheel_name << ", steering_ax = " << steering_axis.transpose() << std::endl;
    
    _log_data.theta_ref = des_theta_1;
    _log_data.steering_axis = steering_axis;
    
    double des_q_1 = q  + (des_theta_1 - theta) * ::sign(steering_axis.z());
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

void SimpleSteering::log(XBot::MatLogger2::Ptr logger)
{
    logger->add(_wheel_name + "_threshold", _comp.getCurrentThreshold());
    logger->add(_wheel_name + "_forward", _log_data.forward);
    logger->add(_wheel_name + "_normal", _log_data.normal);
    logger->add(_wheel_name + "_steering_axis", _log_data.steering_axis);
    logger->add(_wheel_name + "_theta", _log_data.theta);
    logger->add(_wheel_name + "_theta_ref", _log_data.theta_ref);
    logger->add(_wheel_name + "_vdes", _log_data.vdes);
    logger->add(_wheel_name + "_q_des", _prev_qdes);
    logger->add(_wheel_name + "_q", _log_data.q);

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

HysteresisComparator HysteresisComparator::MakeHysteresisComparator(double th_lo, double th_hi, bool init_lo)
{
    HysteresisComparator comp(th_lo, th_hi, init_lo);
    return comp;
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
                                             double max_steering_speed,
                                             std::vector<double> hyst_comp,
                                             std::vector<double> dz_th):
    Task("centauro_steering_" + wheel_name, model->getJointNum()),
    _steering(model, wheel_name, hyst_comp, dz_th),
    _max_steering_dq(max_steering_speed*dt),
    _model(model)
{
    _A.setZero(1, model->getJointNum());
    _b.setZero(1);
    _W.setIdentity(1,1);
    
    // wheel parent link
    auto wheel_parent_name = _model->getUrdf().getLink(wheel_name)->parent_joint->parent_link_name;

    // go up the tree until the steering axis is found
    urdf::JointConstSharedPtr steering_joint;
    auto link = _model->getUrdf().getLink(wheel_parent_name);
    while(!steering_joint && link)
    {
        auto jnt = link->parent_joint;

        if(jnt->type == urdf::Joint::REVOLUTE ||
                jnt->type == urdf::Joint::CONTINUOUS)
        {
            steering_joint = jnt;
            break;
        }

        link = _model->getUrdf().getLink(jnt->parent_link_name);
    }

    // steering joint not found
    if(!steering_joint)
    {
        throw std::runtime_error("steering joint not found for wheel " + wheel_name);
    }

    std::cout << "found steering joint " << steering_joint->name << "\n";

    // re-define wheel parent
    _steering_joint = _model->getJointByName(steering_joint->name);
    
    _model->getJointPosition(_q);
    
    _steering_dof_idx = _model->getDofIndex(steering_joint->name);
    
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
    Eigen::Vector6d wheel_vel;
    _model->getVelocityTwist(_steering.getWheelName(), wheel_vel);

    double q_steering = _steering.computeSteeringAngle(wheel_vel.head<3>());
    double q_current = _q(_steering_dof_idx);
    
    double dq = _lambda*(q_steering - q_current);
    dq = std::min(std::max(dq, -_max_steering_dq), _max_steering_dq);
    
    _b(0) = dq;
    
}

void OpenSoT::tasks::velocity::CentauroAnkleSteering::_log(XBot::MatLogger2::Ptr logger)
{
    _steering.log(logger);
}

