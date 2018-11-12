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
#include <boost/shared_ptr.hpp>
#include <eigen_conversions/eigen_kdl.h>

#define toDeg(X) (X*180.0/M_PI)

Eigen::VectorXd cartesian_utils::computeCapturePoint(const Eigen::VectorXd& com_velocity,
                                                     const Eigen::VectorXd& com_pose)
{
    Eigen::VectorXd cp(3); cp.setZero(3);

    Eigen::VectorXd com_velocity_ = com_velocity;

    double g = 9.81;

    cp[0] = com_pose[0] + com_velocity_[0]*sqrt(com_pose[2]/g);
    cp[1] = com_pose[1] + com_velocity_[1]*sqrt(com_pose[2]/g);
    cp[2] = 0.0;

    return cp;
}

Eigen::VectorXd cartesian_utils::computeFootZMP(const Eigen::VectorXd& forces, const Eigen::VectorXd& torques,
                                                const double d, const double fz_threshold)
{
    Eigen::VectorXd ZMP(3); ZMP.setZero(3);

    if(forces[2] > fz_threshold && fz_threshold >= 0.0){
        ZMP[0] = -1.0 * (torques[1] + forces[0]*d)/forces[2];
        ZMP[1] = (torques[0] - forces[1]*d)/forces[2];
        ZMP[2] = -d;}
    return ZMP;
}

Eigen::VectorXd cartesian_utils::computeZMP(const double Lforce_z, const double Rforce_z,
                           const Eigen::VectorXd& ZMPL, const Eigen::VectorXd& ZMPR,
                           const double fz_threshold)
{
    Eigen::VectorXd ZMP(3); ZMP.setZero(3);

    if((Lforce_z > fz_threshold || Rforce_z > fz_threshold) &&
        fz_threshold >= 0.0){

        ZMP = (ZMPL*Lforce_z + ZMPR*Rforce_z)/(Lforce_z+Rforce_z);
        ZMP[2] = ZMPL[2];
    }
    return ZMP;
}

void  cartesian_utils::computePanTiltMatrix(const Eigen::VectorXd &gaze, KDL::Frame &pan_tilt_matrix)
{
    double pan = std::atan2(gaze[1], gaze[0]);
    double tilt = std::atan2(gaze[2],sqrt(gaze[1]*gaze[1] + gaze[0]*gaze[0]));

    pan_tilt_matrix.Identity();
    pan_tilt_matrix.M.DoRotZ(pan);
    pan_tilt_matrix.M.DoRotY(-tilt);
}

void cartesian_utils::computeCartesianError(const Eigen::MatrixXd &T,
                                  const Eigen::MatrixXd &Td,
                                  Eigen::VectorXd& position_error,
                                  Eigen::VectorXd& orientation_error)
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
    gradient.setZero(x.rows());
    Eigen::VectorXd deltas(x.rows());
    deltas.setZero(x.rows());
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

Eigen::MatrixXd cartesian_utils::computeHessian(const Eigen::VectorXd &x,
                                                   GradientVector& vec,
                                                   const double& step) {
    Eigen::MatrixXd hessian(vec.size(),x.size());
    Eigen::VectorXd deltas(x.rows());
    deltas.setZero(x.rows());
    const double h = step;
    for(unsigned int i = 0; i < vec.size(); ++i)
    {
        deltas[i] = h;
        Eigen::VectorXd gradient_a = vec.compute(x+deltas);
        Eigen::VectorXd gradient_b = vec.compute(x-deltas);
        Eigen::VectorXd gradient(vec.size());
        for(unsigned int j = 0; j < vec.size(); ++j)
            gradient[j] = (gradient_a[j] - gradient_b[j])/(2.0*h);

        hessian.block(0,i,hessian.rows(),1) = gradient;

        //hessian.setCol(i,gradient);
        deltas[i] = 0.0;
    }

    return hessian;
}

