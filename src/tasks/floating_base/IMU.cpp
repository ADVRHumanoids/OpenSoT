#include <OpenSoT/tasks/floating_base/IMU.h>
#include <XBotInterface/ImuSensor.h>

OpenSoT::tasks::floating_base::IMU::IMU(XBot::ModelInterface &robot, XBot::ImuSensor::ConstPtr imu):
    Task("IMU", 6),_robot(robot), _imu(imu)
{
    _hessianType = HST_SEMIDEF;

    _W.setIdentity(3, 3);

    _robot.getFloatingBaseLink(_fb_link);

    std::string imu_link;
    imu_link = _imu->getSensorName();

    Eigen::MatrixXd tmp;
    robot.getJacobian(_fb_link, imu_link, tmp);
    if(tmp.norm() < 1e-6)
        throw std::runtime_error("Imu is not attached in floating_base link!");


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
