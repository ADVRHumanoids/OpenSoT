#include <OpenSoT/constraints/velocity/AccelerationLimits.h>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

using namespace OpenSoT::constraints::velocity;

AccelerationLimits::AccelerationLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDotLimit,
                                       const double dT)
    : Constraint("acceleration_limits", robot.getNv()), _robot(robot), _dT(dT)
{

    if (qDDotLimit.size() != _x_size)
        throw std::runtime_error("qDDotLimit.size() != _x_size()");

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

    this->setAccelerationLimits(qDDotLimit);
}

void AccelerationLimits::update()
{
    // the joint velocities are obtained from the robot model
    // these could be either the measured values, or commanded values
    const Eigen::VectorXd& v = _robot.getJointVelocity();

    assert(_qDDotLimit.size() == _x_size);
    for (unsigned int i = 0; i < _qDDotLimit.size(); ++i)
    {
        // Note: assume optimized joint velocity is in rad/second
        // this interpertation is different from the rest of OpenSoT which assumes rad/timestep
        _lowerBound[i] = (-1.0 * _qDDotLimit_dTScaled[i] + v[i]);
        _upperBound[i] = (1.0 * _qDDotLimit_dTScaled[i] + v[i]);
    }
}

Eigen::VectorXd AccelerationLimits::getLowerBound() const
{
    return _lowerBound;
}

Eigen::VectorXd AccelerationLimits::getUpperBound() const
{
    return _upperBound;
}

double AccelerationLimits::getDT()
{
    return _dT;
}

Eigen::VectorXd AccelerationLimits::getAccelerationLimits()
{
    return _qDDotLimit;
}

void AccelerationLimits::setAccelerationLimits(const Eigen::VectorXd &qDDotLimit)
{
    _qDDotLimit = qDDotLimit;
    _qDDotLimit_dTScaled = qDDotLimit.array().abs() * _dT;
    this->update();
}
