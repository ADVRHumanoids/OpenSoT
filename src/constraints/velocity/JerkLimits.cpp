#include <OpenSoT/constraints/velocity/JerkLimits.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

using namespace OpenSoT::constraints::velocity;

JerkLimits::JerkLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDDotLimit, const double dT)
    : Constraint("jerk_limits", robot.getNv()), _robot(robot), _dT(dT)
{

    if (qDDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    this->setJerkLimits(qDDDotLimit);
}

void JerkLimits::update()
{
    // the joint velocity and acceleration are obtained from the robot model
    // these could be either the measured values, or commanded values
    const Eigen::VectorXd& v = _robot.getJointVelocity();
    const Eigen::VectorXd& a = _robot.getJointAcceleration();
    assert(_qDDDotLimit.size() == _x_size);

    for (unsigned int i = 0; i < _qDDDotLimit.size(); ++i)
    {
        // Note: assume optimized joint velocity is in rad/second
        // this interpertation is different from the rest of OpenSoT which assumes rad/timestep
        _lowerBound[i] = ((-1.0 * _qDDDotLimit_dTScaled[i] + a[i]) * _dT + v[i]);
        _upperBound[i] = ((1.0 * _qDDDotLimit_dTScaled[i] + a[i]) * _dT + v[i]);
    }
}

double JerkLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd JerkLimits::getJerkLimits()
{
    return _qDDDotLimit;
}

void JerkLimits::setJerkLimits(const Eigen::VectorXd &qDDDotLimit)
{
    _qDDDotLimit = qDDDotLimit;
    _qDDDotLimit_dTScaled = qDDDotLimit.array().abs() * _dT;
    this->update();
}

Eigen::VectorXd JerkLimits::getLowerBound() const
{
    return _lowerBound;
}

Eigen::VectorXd JerkLimits::getUpperBound() const
{
    return _upperBound;
}