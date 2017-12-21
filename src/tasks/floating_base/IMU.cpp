#include <OpenSoT/tasks/floating_base/IMU.h>
#include <XBotInterface/ImuSensor.h>

OpenSoT::tasks::floating_base::IMU::IMU(XBot::ModelInterface &robot, XBot::ImuSensor::ConstPtr imu):
    Task("IMU", 6),_robot(robot), _imu(imu)
{
    _hessianType = HST_SEMIDEF;

    _W.setIdentity(3, 3);

    _robot.getFloatingBaseLink(_fb_link);

    _update(Eigen::VectorXd::Zero(robot.getJointNum()));
}

OpenSoT::tasks::floating_base::IMU::~IMU()
{

}

void OpenSoT::tasks::floating_base::IMU::_update(const Eigen::VectorXd &x)
{
    _robot.getJacobian(_fb_link, _J);
    _imu->getAngularVelocity(_angular_velocity);

    _A = _J.block<3,6>(3,0);

    Eigen::Matrix3d R;
    _robot.getOrientation(_imu->getSensorName(), R);
    _b = R*_angular_velocity;
}
