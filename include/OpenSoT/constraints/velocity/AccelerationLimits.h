#ifndef __BOUNDS_VELOCITY_ACCELERATIONLIMITS_H__
#define __BOUNDS_VELOCITY_ACCELERATIONLIMITS_H__

#include <Eigen/Dense>
#include <OpenSoT/Constraint.h>
#include <fstream>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT
{
namespace constraints
{
namespace velocity
{
/**
 * @brief The AccelerationLimits class implements a bound on joint accelerations
 */
class AccelerationLimits : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{
  public:
    typedef std::shared_ptr<AccelerationLimits> Ptr;

  private:
    const XBot::ModelInterface &_robot;
    Eigen::VectorXd _qDDotLimit;
    Eigen::VectorXd _qDDotLimit_dTScaled;
    double _dT;

  public:
    /**
     * @brief AccelerationLimits constructor
     * @param qDDotLimit the joint acceleration limit. It is always a positive number [rad/s^2]
     * @param dT the time constant at which we are performing velocity control [s]
     * @param x_size the size of the unknowns that we want to bound (it CANNOT be a subset)
     */
    AccelerationLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDotLimit, const double dT);

    /**
     * @brief getLowerBound returns the current lower bounds on velocity command.
     * @return lower limit imposed by the constraint.
     */
    Eigen::VectorXd getLowerBound() const;

    /**
     * @brief getUpperBound returns the current upper bounds on velocity command.
     * @return upper limit imposed by the constraint.
     */
    Eigen::VectorXd getUpperBound() const;

    /**
     * @brief getAccelerationLimits returns the current acceleration limits.
     * @return the joint acceleration limits. It is always a positive double [rad/s^2]
     */
    Eigen::VectorXd getAccelerationLimits();
    
    /**
     * @brief setAccelerationLimits
     * @param qDDotLimit the joint acceleration limits. It needs be a positive number [rad/s^2]
     */
    void setAccelerationLimits(const Eigen::VectorXd &qDDotLimit);

    /**
     * @brief getDT returns the (constant) sample time we assume on the system.
     * @return the system sample time in [s]
     */
    double getDT();
    void update();
};
} // namespace velocity
} // namespace constraints
} // namespace OpenSoT

#endif
