#include "cartesian_utils.h"

void cartesian_utils::homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                              const double x, const double y, const double z,
                              const double R, const double P, const double Y)
{
    KDL::Frame tmp(KDL::Rotation::RPY(R, P, Y), KDL::Vector(x, y, z));

    T.resize(4,4);
    T.eye();

    T(0,3) = tmp.p.x();
    T(1,3) = tmp.p.y();
    T(2,3) = tmp.p.z();

    T(0,0) = tmp.M.UnitX().x(); T(0,1) = tmp.M.UnitY().x(); T(0,2) = tmp.M.UnitZ().x();
    T(1,0) = tmp.M.UnitX().y(); T(1,1) = tmp.M.UnitY().y(); T(1,2) = tmp.M.UnitZ().y();
    T(2,0) = tmp.M.UnitX().z(); T(2,1) = tmp.M.UnitY().z(); T(2,2) = tmp.M.UnitZ().z();
}

void cartesian_utils::computeCartesianError(const yarp::sig::Matrix &T,
                                            const yarp::sig::Matrix &Td,
                                            yarp::sig::Vector& position_error,
                                            yarp::sig::Vector& orientation_error)
{
    position_error.resize(3, 0.0);
    orientation_error.resize(3, 0.0);

    KDL::Frame x; // ee pose
    x.Make4x4((double*)T.data());
    quaternion q;
    x.M.GetQuaternion(q.x, q.y, q.z, q.w);

    KDL::Frame xd; // ee desired pose
    xd.Make4x4((double*)Td.data());
    quaternion qd;
    xd.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    xerr_p = x.p - xd.p;
    xerr_o = quaternion::error(q, qd);

    position_error[0] = xerr_p[0];
    position_error[1] = xerr_p[1];
    position_error[2] = xerr_p[2];

    orientation_error[0] = xerr_o[0];
    orientation_error[1] = xerr_o[1];
    orientation_error[2] = xerr_o[2];
}
