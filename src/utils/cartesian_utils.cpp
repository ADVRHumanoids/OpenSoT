/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi,
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <OpenSoT/utils/cartesian_utils.h>
#include <memory>
#include <eigen_conversions/eigen_kdl.h>

#define toDeg(X) (X*180.0/M_PI)


void  cartesian_utils::computePanTiltMatrix(const Eigen::VectorXd &gaze, KDL::Frame &pan_tilt_matrix)
{
    double pan = std::atan2(gaze[1], gaze[0]);
    double tilt = std::atan2(gaze[2],sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

    pan_tilt_matrix.Identity();
    pan_tilt_matrix.M.DoRotZ(pan);
    pan_tilt_matrix.M.DoRotY(-tilt);
}

void cartesian_utils::computeCartesianError(const Eigen::Matrix4d &T,
                                  const Eigen::Matrix4d &Td,
                                  Eigen::Vector3d& position_error,
                                  Eigen::Vector3d& orientation_error)
{
    position_error.setZero(3);
    orientation_error.setZero(3);

    KDL::Frame x; // ee pose
    x.Identity();
    Eigen::Matrix4d tmp = T;
    tf::transformEigenToKDL(Eigen::Affine3d(tmp),x);
    quaternion q;
    x.M.GetQuaternion(q.x, q.y, q.z, q.w);

    KDL::Frame xd; // ee desired pose
    xd.Identity();
    tmp = Td;
    tf::transformEigenToKDL(Eigen::Affine3d(tmp),xd);
    quaternion qd;
    xd.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    //This is needed to move along the short path in the quaternion error
    if(quaternion::dot(q, qd) < 0.0)
        q = q.operator *(-1.0); //che cagata...

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    xerr_p = xd.p - x.p;
    xerr_o = quaternion::error(q, qd);


    position_error(0) = xerr_p.x();
    position_error(1) = xerr_p.y();
    position_error(2) = xerr_p.z();

    orientation_error(0) = xerr_o.x();
    orientation_error(1) = xerr_o.y();
    orientation_error(2) = xerr_o.z();
}

void cartesian_utils::computeCartesianError(const Eigen::Affine3d &T,
                                  const Eigen::Affine3d &Td,
                                  Eigen::Vector3d& position_error,
                                  Eigen::Vector3d& orientation_error)
{
    Eigen::Quaterniond q(T.rotation());
    Eigen::Quaterniond qd(Td.rotation());

    position_error = Td.translation()- T.translation();

    //This is needed to move along the short path in the quaternion error
    if(q.dot(qd) < 0.0)
        orientation_error = quaternion::error(-q.x(), -q.y(), -q.z(), -q.w(),
                                              qd.x(), qd.y(), qd.z(), qd.w());
    else
        orientation_error = quaternion::error(q.x(),  q.y(),  q.z(),  q.w(),
                                              qd.x(), qd.y(), qd.z(), qd.w());
}

Eigen::VectorXd cartesian_utils::computeGradient(const Eigen::VectorXd &x,
                                                    CostFunction& fun,
                                                    const double& step) {
    std::vector<bool> jointMask(x.size(), true);
    return computeGradient(x, fun, jointMask, step);
}

Eigen::VectorXd cartesian_utils::computeGradient(const Eigen::VectorXd &x,
                                                    CostFunction& fun,
                                                    const std::vector<bool>& jointMask,
                                                    const double& step) {
    Eigen::VectorXd gradient(x.rows());
    gradient.setZero();
    Eigen::VectorXd deltas(x.rows());
    deltas.setZero();
    assert(jointMask.size() == x.size() &&
           "jointMask must have the same size as x");
    const double h = step;
    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        if(jointMask[i])
        {
            deltas[i] = h;
            double fun_a = fun.compute(x+deltas);
            double fun_b = fun.compute(x-deltas);

            gradient[i] = (fun_a - fun_b)/(2.0*h);
            deltas[i] = 0.0;
        } else
            gradient[i] = 0.0;
    }

    return gradient;
}


