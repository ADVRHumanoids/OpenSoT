#ifndef _TASK_SOLVER_H_
#define _TASK_SOLVER_H_

#include <qpOASES.hpp>
#include <yarp/sig/all.h>

class task_solver
{
public:
    static bool computeControlHQP(const yarp::sig::Matrix &J0,
                                  const yarp::sig::Vector &e0,
                                  const yarp::sig::Matrix& J1,
                                  const yarp::sig::Vector& e1,
                                  const yarp::sig::Matrix& J2,
                                  const yarp::sig::Vector& eq,
                                  const qpOASES::HessianType t2HessianType,
                                  const yarp::sig::Vector &qMax,
                                  const yarp::sig::Vector &qMin,
                                  const yarp::sig::Vector &q,
                                  const double &_maxJointVelocity,
                                  const double &_dT,
                                  yarp::sig::Vector &dq_ref);

    static bool computeControlHQP(const yarp::sig::Matrix &J0,
                                  const yarp::sig::Vector &e0,
                                  const yarp::sig::Matrix &J1,
                                  const yarp::sig::Vector &eq,
                                  const qpOASES::HessianType t1HessianType,
                                  const yarp::sig::Vector &qMax,
                                  const yarp::sig::Vector &qMin,
                                  const yarp::sig::Vector &q,
                                  const double &_maxJointVelocity,
                                  const double &_dT,
                                  yarp::sig::Vector &dq_ref);
};

#endif
