#ifndef _CARTESIAN_UTILS_H_
#define _CARTESIAN_UTILS_H_

#include <yarp/sig/all.h>
#include <kdl/frames.hpp>

class quaternion
{
public:
    double x;
    double y;
    double z;
    double w;

    quaternion():
        x(0.0),
        y(0.0),
        z(0.0),
        w(1.0)
    {

    }

    quaternion(double x, double y, double z, double w);

    KDL::Rotation skew()
    {
        KDL::Rotation s(0.0,  -z,   y,
                          z, 0.0,  -x,
                         -y,   x, 0.0);
        return s;
    }

    static KDL::Vector error(quaternion& q, quaternion& qd)
    {
        KDL::Vector e(0.0, 0.0, 0.0);

        KDL::Vector eps(q.x, q.y, q.z);
        KDL::Vector epsd(qd.x, qd.y, qd.z);

        e = qd.w*eps - q.w*epsd + qd.skew()*eps;

        return e;
    }
};

class cartesian_utils
{
    static void computeCartesianError(const yarp::sig::Matrix &T,
                                      const yarp::sig::Matrix &Td,
                                      yarp::sig::Vector& position_error,
                                      yarp::sig::Vector& orientation_error);
    static void homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                                         const double x, const double y, const double z,
                                         const double R, const double P, const double Y);
};

#endif
