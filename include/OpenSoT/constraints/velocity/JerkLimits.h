#ifndef __BOUNDS_VELOCITY_JERKLIMITS_H__
#define __BOUNDS_VELOCITY_JERKLIMITS_H__

#include <Eigen/Dense>
#include <OpenSoT/Constraint.h>
#include <xbot2_interface/xbotinterface2.h>

namespace OpenSoT
{
namespace constraints
{
namespace velocity
{
/**
 * @brief The JerkLimits class implements a bound on joint jerks
 */
class JerkLimits : public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{
  public:
    typedef std::shared_ptr<JerkLimits> Ptr;

  private:
    const XBot::ModelInterface &_robot;
    Eigen::VectorXd _qDDDotLimit;
    Eigen::VectorXd _qDDDotLimit_dTScaled;
    double _dT;

  public:
    /**
     * @brief JerkLimits constructor
     * @param qDDDotLimit the joint jerk limit. It is always a positive number [rad/s^3]
     * @param dT the time constant at which we are performing velocity control [s]
     * @param x_size the size of the unknowns that we want to bound (it CANNOT be a subset)
     */
    JerkLimits(const XBot::ModelInterface &robot, const Eigen::VectorXd &qDDDotLimit, const double dT);

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
     * @brief getJerkLimits returns the current jerk limits.
     * @return the joint jerk limits. It is always a positive double [rad/s^3]
     */
    Eigen::VectorXd getJerkLimits();

    
    /**
     * @brief setJerkLimits
     * @param qDDDotLimit the joint jerk limits. It needs be a positive number [rad/s^3]
     */
    void setJerkLimits(const Eigen::VectorXd &qDDDotLimit);

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
