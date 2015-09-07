#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <yarp/math/Math.h>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

Dynamics::Dynamics(const yarp::sig::Vector &q, const yarp::sig::Vector &q_dot,
                   const yarp::sig::Vector &jointTorquesMax, iDynUtils &robot_model,
                   const double dT):
    Constraint(q.size()),
    _jointTorquesMin(-1.0*jointTorquesMax),
    _jointTorquesMax(jointTorquesMax),
    _robot_model(robot_model),
    _dT(dT),
    _b(q.size()),
    _M(6+q.size(), 6+q.size())
{
    _Aineq.resize(_x_size, _x_size);
    _bLowerBound.resize(_x_size);
    _bUpperBound.resize(_x_size);

    /**
     * Since the constraint is specified in velocity we approximate the acceleration
     * to the first order, for these reason the jointTorquesMin/Max has to be multiplied by dT
    **/
    _jointTorquesMax = _dT*_jointTorquesMax;
    _jointTorquesMin = _dT*_jointTorquesMin;

    update(yarp::math::cat(q, q_dot));
}

void Dynamics::update(const yarp::sig::Vector &x)
{
    assert(x.size() == 2*_x_size);

    /**
     * Here I suppose that q and dq have been set in the iDynTree Model,
     * this basically save time since I do not have to compute anything more than ID:
     *
     *      ID(q, dq, zero) = C(q,dq)dq + g(q) = -b1
    **/
    _b = _robot_model.iDyn3_model.getTorques();

    /**
     * Since the constraint is specified in velocity we approximate the acceleration
     * to the first order, for these reason b1 has to be multiplied by dT
    **/
    _b = -1.0*_dT*_b;

    /**
     * Here we need the Mass matrix to compute the term:
     *
     *      Mdq = b2
     *
     * and summed to the previous one
    **/
    _M.resize(6+_x_size, 6+_x_size);
    _robot_model.iDyn3_model.getFloatingBaseMassMatrix(_M);
    _M = _M.removeCols(0,6); _M = _M.removeRows(0,6);
    _b = _b + _M*x.subVector(_x_size, (2*_x_size)-1);

    /**
     * The final constraint is given by:
    **/
    _bLowerBound = _jointTorquesMin + _b;
    _bUpperBound = _jointTorquesMax + _b;
    _Aineq = _M/_dT;
}
