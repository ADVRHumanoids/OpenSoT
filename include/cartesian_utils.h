#ifndef _CARTESIAN_UTILS_H_
#define _CARTESIAN_UTILS_H_

#include <yarp/sig/all.h>
#include <kdl/frames.hpp>

/**
  This class implements quaternion error as in the paper:
    "Operational Space Control: A Theoretical and Empirical Comparison"
  Authors: Jun Nakanishi, Rick Cory, Michael Mistry, Jan Peters and Stefan Schaal
  The International Journal of Robotics Research, Vol. 27, No. 6, June 2008, pp. 737–757
  **/
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

    quaternion(double _x, double _y, double _z, double _w):
        x(_x),
        y(_y),
        z(_z),
        w(_w)
    {

    }

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
public:
    static void computeCartesianError(yarp::sig::Matrix &T,
                                      yarp::sig::Matrix &Td,
                                      yarp::sig::Vector& position_error,
                                      yarp::sig::Vector& orientation_error);
    static void homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                                         const double x, const double y, const double z,
                                         const double R, const double P, const double Y);
    static void homogeneousMatrixFromQuaternion(yarp::sig::Matrix& T,
                                                const double x, const double y, const double z,
                                                const double quaternion_x, const double quaternion_y, const double quaternion_z, const double quaternion_w);
    static void fromKDLFrameToYARPMatrix(const KDL::Frame& Ti, yarp::sig::Matrix& To);
    static void fromYARPMatrixtoKDLFrame(const yarp::sig::Matrix& Ti, KDL::Frame& To);
    static void printHomogeneousTransform(const yarp::sig::Matrix& T);
};

#endif
