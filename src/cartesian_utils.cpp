/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "cartesian_utils.h"

#define toDeg(X) (X*180.0/M_PI)

void cartesian_utils::homogeneousMatrixFromRPY(yarp::sig::Matrix& T,
                              const double x, const double y, const double z,
                              const double R, const double P, const double Y)
{
    KDL::Frame tmp(KDL::Rotation::RPY(R, P, Y), KDL::Vector(x, y, z));

    fromKDLFrameToYARPMatrix(tmp, T);
}

void cartesian_utils::homogeneousMatrixFromQuaternion(yarp::sig::Matrix &T,
                                                      const double x, const double y, const double z,
                                                      const double quaternion_x, const double quaternion_y, const double quaternion_z, const double quaternion_w)
{
    KDL::Frame tmp(KDL::Rotation::Quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w), KDL::Vector(x, y, z));

    fromKDLFrameToYARPMatrix(tmp, T);
}

void cartesian_utils::computeCartesianError(yarp::sig::Matrix &T,
                                            yarp::sig::Matrix &Td,
                                            yarp::sig::Vector& position_error,
                                            yarp::sig::Vector& orientation_error)
{
    position_error.resize(3, 0.0);
    orientation_error.resize(3, 0.0);

    KDL::Frame x(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)); // ee pose
    fromYARPMatrixtoKDLFrame(T, x);
    quaternion q;
    x.M.GetQuaternion(q.x, q.y, q.z, q.w);

    KDL::Frame xd(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)); // ee desired pose
    fromYARPMatrixtoKDLFrame(Td, xd);
    quaternion qd;
    xd.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    xerr_p = xd.p - x.p;
    xerr_o = quaternion::error(q, qd);

    position_error[0] = xerr_p.x();
    position_error[1] = xerr_p.y();
    position_error[2] = xerr_p.z();

    orientation_error[0] = xerr_o.x();
    orientation_error[1] = xerr_o.y();
    orientation_error[2] = xerr_o.z();
}

void cartesian_utils::fromKDLFrameToYARPMatrix(const KDL::Frame &Ti, yarp::sig::Matrix &To)
{
    To.resize(4,4);
    To.eye();
    To(0,0) = Ti.M.UnitX().x(); To(0,1) = Ti.M.UnitY().x(); To(0,2) = Ti.M.UnitZ().x(); To(0,3) = Ti.p.x();
    To(1,0) = Ti.M.UnitX().y(); To(1,1) = Ti.M.UnitY().y(); To(1,2) = Ti.M.UnitZ().y(); To(1,3) = Ti.p.y();
    To(2,0) = Ti.M.UnitX().z(); To(2,1) = Ti.M.UnitY().z(); To(2,2) = Ti.M.UnitZ().z(); To(2,3) = Ti.p.z();
}

void cartesian_utils::fromYARPMatrixtoKDLFrame(const yarp::sig::Matrix &Ti, KDL::Frame &To)
{
    To.p.data[0] = Ti(0,3); To.p.data[1] = Ti(1,3); To.p.data[2] = Ti(2,3);
    To.M.data[0] = Ti(0,0); To.M.data[1] = Ti(0,1); To.M.data[2] = Ti(0,2);
    To.M.data[3] = Ti(1,0); To.M.data[4] = Ti(1,1); To.M.data[5] = Ti(1,2);
    To.M.data[6] = Ti(2,0); To.M.data[7] = Ti(2,1); To.M.data[8] = Ti(2,2);
}

void cartesian_utils::printHomogeneousTransform(const yarp::sig::Matrix &T)
{
    KDL::Frame p(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)); // ee desired pose
    fromYARPMatrixtoKDLFrame(T, p);

    double R = 0.0;
    double P = 0.0;
    double Y = 0.0;
    p.M.GetRPY(R,P,Y);

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    p.M.GetQuaternion(qx, qy, qz, qw);

    std::cout<<"Position: [ "<<p.p.x()<<" "<<p.p.y()<<" "<<p.p.z()<<" ] [m]"<<std::endl;
    std::cout<<"RPY: [ "<<toDeg(R)<<" "<<toDeg(P)<<" "<<toDeg(Y)<<" ] [deg]"<<std::endl;
    std::cout<<"Quaternion: [ "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<" ]"<<std::endl;
}
