#ifndef _TASK_SOLVER_H_
#define _TASK_SOLVER_H_

#include <qpOASES.hpp>
#include <yarp/sig/all.h>
#include <ros/ros.h>

class task_solver
{
public:
    /**
     * @brief computeControlHQP solve a linear optimization problem with three stacked tasks
     * in the form:
     *      min||Ax - b||
     *       st. c1 < x < c2
     *           Dx = d
     * @param J0 A matrix of first task
     * @param e0 b vector of first task
     * @param J1 A matrix of second task
     * @param e1 b vector of second task
     * @param J2 A matrix of third task
     * @param eq b vector of third task
     * @param t2HessianType
     * @param qMax Bound max on joint limits
     * @param qMin Bound min on joint limits
     * @param q Actual q
     * @param _maxJointVelocity Bound on joint velocities
     * @param _dT Control time step
     * @param dq_ref Unknown
     * @param velocity_bounds_scale Scale factor used to reduce velocity in all the tasks above the last one!
     * @return
     */
    static bool computeControlHQP(const yarp::sig::Matrix &J0,
                                  const yarp::sig::Vector &e0,
                                  const yarp::sig::Matrix &J1,
                                  const yarp::sig::Vector &e1,
                                  const yarp::sig::Matrix &J2,
                                  const yarp::sig::Vector &eq,
                                  const qpOASES::HessianType t2HessianType,
                                  const yarp::sig::Vector &qMax,
                                  const yarp::sig::Vector &qMin,
                                  const yarp::sig::Vector &q,
                                  const double &_maxJointVelocity,
                                  const yarp::sig::Matrix &JCoM, const double &_maxCoMVelocity,
                                  const yarp::sig::Matrix &cartesian_A, const yarp::sig::Vector &cartesian_b,
                                  const double &_dT,
                                  yarp::sig::Vector &dq_ref,
                                  const double velocity_bounds_scale);

    /**
     * @brief computeControlHQP solve a linear optimization problem with two stacked tasks
     * in the form:
     *      min||Ax - b||
     *       st. c1 < x < c2
     *           Dx = d
     * @param J0 A matrix of first task
     * @param e0 b vector of first task
     * @param J1 A matrix of second task
     * @param eq b vector of second task
     * @param t1HessianType
     * @param qMax Bound max on joint limits
     * @param qMin Bound min on joint limits
     * @param q Actual q
     * @param _maxJointVelocity Bound on joint velocities
     * @param _dT Control time step
     * @param dq_ref Unknown
     * @param velocity_bounds_scale Scale factor used to reduce velocity in all the tasks above the last one!
     * @return
     */
    static bool computeControlHQP(const yarp::sig::Matrix &J0,
                                  const yarp::sig::Vector &e0,
                                  const yarp::sig::Matrix &J1,
                                  const yarp::sig::Vector &eq,
                                  qpOASES::HessianType t1HessianType,
                                  const yarp::sig::Vector &qMax,
                                  const yarp::sig::Vector &qMin,
                                  const yarp::sig::Vector &q,
                                  const double &MAX_JOINT_VELOCITY,
                                  const yarp::sig::Matrix &JCoM,
                                  const double &MAX_COM_VELOCITY,
                                  const yarp::sig::Matrix &cartesian_A, const yarp::sig::Vector &cartesian_b,
                                  const double &dT,
                                  yarp::sig::Vector &dq_ref,
                                  const double velocity_bounds_scale);
};

#endif
