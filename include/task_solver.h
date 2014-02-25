#ifndef _TASK_SOLVER_H_
#define _TASK_SOLVER_H_

#include <qpOASES.hpp>
#include <yarp/sig/all.h>

class task_solver
{
public:
    static bool computeControlHQP(const yarp::sig::Matrix& J1,
                                  const yarp::sig::Vector& e1,
                                  const yarp::sig::Matrix& J2,
                                  const yarp::sig::Vector& eq,
                                  const yarp::sig::Vector &qMax,
                                  const yarp::sig::Vector &qMin,
                                  const yarp::sig::Vector &q,
                                  const double &MAX_JOINT_VELOCITY,
                                  const double &dT, yarp::sig::Vector &dq_ref);
};

#endif
