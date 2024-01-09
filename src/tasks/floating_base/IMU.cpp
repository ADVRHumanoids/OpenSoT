#include <OpenSoT/tasks/floating_base/IMU.h>

OpenSoT::tasks::floating_base::IMU::IMU(XBot::ModelInterface &robot, XBot::ImuSensor::ConstPtr imu):
    Task("IMU", 6),_robot(robot), _imu(imu)
{
    _hessianType = HST_SEMIDEF;

    _W.setIdentity(6, 6);

    _b.setZero(6);

    _A.setZero(6,6);

    _robot.getFloatingBaseLink(_fb_link);

    std::string imu_link;
    imu_link = _imu->getName();

    Eigen::MatrixXd tmp;
    robot.getRelativeJacobian(_fb_link, imu_link, tmp);
    if(tmp.norm() < 1e-6)
        throw std::runtime_error("Imu is not attached to floating_base link!");


    _update(Eigen::VectorXd::Zero(robot.getJointNum()));
}

OpenSoT::tasks::floating_base::IMU::~IMU()
{

}

void OpenSoT::tasks::floating_base::IMU::_update(const Eigen::VectorXd &x)
{
    _robot.getJacobian(_fb_link, _J);
    _imu->getAngularVelocity(_angular_velocity);

    _A.block(3,0,3,6) = _J.block<3,6>(3,0);

    Eigen::Matrix3d R = _robot.getPose(_imu->getName()).linear();
    _b.tail(3) = R*_angular_velocity;
}
