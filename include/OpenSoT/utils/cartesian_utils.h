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

#ifndef _CARTESIAN_UTILS_H__
#define _CARTESIAN_UTILS_H__

#include <kdl/frames.hpp>
#include <vector>
#include <list>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

/**
 * @brief The CostFunction class pure virtual function used to describe functions for computeGradient method.
 */
class CostFunction {
public:
    /**
     * @brief compute value of function in x
     * @param x
     * @return scalar
     */

    virtual double compute(const Eigen::VectorXd &x) = 0;
};

/**
 * @brief The GradientVector class pure virtual function used to describe functions for computeHessian method.
 */
class GradientVector {
    int _size;
public:
    GradientVector(const int x_size) : _size(x_size) {}
    /**
     * @brief compute value of function in x
     * @param x
     * @return scalar
     */

    virtual Eigen::VectorXd compute(const Eigen::VectorXd &x) = 0;
    int size() { return _size; }
};

/**
  This class implements quaternion error as in the paper:
    "Operational Space Control: A Theoretical and Empirical Comparison"
  Authors: Jun Nakanishi, Rick Cory, Michael Mistry, Jan Peters and Stefan Schaal
  The International Journal of Robotics Research, Vol. 27, No. 6, June 2008, pp. 737–757

  REMEMBER: if e is the quaternion error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
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

    /**
     * @brief dot product between two quaternions
     * @param a first quaternion
     * @param b second quaternion
     * @return a scalar
     */
    static double dot(const quaternion& a, const quaternion& b)
    {
        return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    }

    /**
     * @brief operator * product between a quaternion and a scalar
     * @param a scalar
     * @return a quaternion
     */
    quaternion operator*(const double a)
    {
        quaternion q(x, y, z, w);

        q.x *= a;
        q.y *= a;
        q.z *= a;
        q.w *= a;

        return q;
    }

    /**
     * @brief skew operator
     * @return the skew matrix of the quaternion
     */
    KDL::Rotation skew()
    {
        KDL::Rotation s(0.0,  -z,   y,
                          z, 0.0,  -x,
                         -y,   x, 0.0);
        return s;
    }

    /**
     * @brief error compute the error between two quaternion to be usable in orientation control
     * @param q actual quaternion
     * @param qd desired quaternion
     * @return an error vector [3x1]
     *
     * REMEMBER: if e is the quaternion error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
     *
     */
    static KDL::Vector error(quaternion& q, quaternion& qd)
    {
        KDL::Vector e(0.0, 0.0, 0.0);

        KDL::Vector eps(q.x, q.y, q.z);
        KDL::Vector epsd(qd.x, qd.y, qd.z);

        e = qd.w*eps - q.w*epsd + qd.skew()*eps;

        return e;
    }

    static Eigen::Vector3d error(const double& qx,const double& qy,const double& qz,const double& qw,
                                 const double& qdx,const double& qdy,const double& qdz,const double& qdw)
    {
        Eigen::Vector3d e(0.0, 0.0, 0.0);

        Eigen::Vector3d eps(qx, qy, qz);
        Eigen::Vector3d epsd(qdx, qdy, qdz);

        Eigen::Matrix3d skew;
        skew<<  0.0,  -qdz,  qdy,
             qdz,   0.0, -qdx,
            -qdy, qdx,    0.0;

        e = qdw*eps - qw*epsd + skew*eps;

        return e;
    }

    /**
     * @brief normalize a given quaternion:
     * Given q = (x, y, z, w)
     * return q_n = (x/d, y/d, z/d, w/d)
     * with d = sqrt(x**2 + y**2 + z**2 + w**2)
     * @param q quaternion to normalize
     */
    static void normalize(quaternion& q)
    {
        double d = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        q.x = q.x/d;
        q.y = q.y/d;
        q.z = q.z/d;
        q.w = q.w/d;
    }
};

/**
 * LDLTInverse provide a simple template Eigen-based (implemented using LDLT) class to compute Inverse
 * NOTE: FAST, works with squared Positive/Negative-SemiDefinite matrices, RT-safe
 */
template<class _Matrix_Type_> class LDLTInverse
{
public:
    LDLTInverse(const _Matrix_Type_ &a)
    {
        I.resize(a.rows(), a.cols()); I.setIdentity(I.rows(), I.cols());
    }
    
    void compute(const _Matrix_Type_ &a, _Matrix_Type_ &ainv)
    {
        LDLT.compute(a);
        ainv = LDLT.solve(I);
    }
    
private:
    Eigen::LDLT<_Matrix_Type_> LDLT;
    _Matrix_Type_ I;
};



/**
 * SVDPseudoInverse provide a simple template Eigen-based (implemented using SVD) class to compute pseudo inverse
 * NOTE: SLOW, works with any matrix, NON RT-safe
 */
template<class _Matrix_Type_> class SVDPseudoInverse
{
public:
    SVDPseudoInverse(const _Matrix_Type_ &a, const double epsilon = std::numeric_limits<double>::epsilon()):
        _epsilon(epsilon),
        _svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV)
    {

    }

    void compute(const _Matrix_Type_ &a, _Matrix_Type_ &ainv, const double epsilon = std::numeric_limits<double>::epsilon())
    {
        _svd.compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        _singularValues = _svd.singularValues().array().abs();
        
        _tolerance = epsilon * std::max(a.cols(), a.rows()) *_singularValues(0);
        
        for(unsigned int i = 0; i < _singularValues.size(); ++i)
        {
            if(_singularValues[i] < _tolerance)
                _singularValues[i] = 0.0;
            else
                _singularValues[i] = 1./_singularValues[i];
        }
        
        _tmp = _svd.matrixV() *  _singularValues.asDiagonal();
        ainv.noalias() =  _tmp * _svd.matrixU().transpose();
    }

    void setEpsilon(const double epsilon)
    {
        _epsilon = epsilon;
    }

private:
    double _epsilon;
    Eigen::JacobiSVD<_Matrix_Type_> _svd;
    double _tolerance;
    
    Eigen::VectorXd _singularValues;
    Eigen::MatrixXd _tmp;
};

class cartesian_utils
{
public:
    /**
     * @brief pnpoly this code is EXACTLY the code of the PNPOLY - Point Inclusion in Polygon Test
        W. Randolph Franklin (WRF) to test if a point is inside a plygon
        (https://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html)
     * @param nvert Number of vertices in the polygon
     * @param vertx Arrays containing the x-coordinates of the polygon's vertices
     * @param verty Arrays containing the y-coordinates of the polygon's vertices
     * @param testx X-coordinate of the test point
     * @param testy Y-coordinate of the test point
     * @return
     *          -1 if the point is outside of the
     *           0 if the point is on an edge or at a vertes
     *           1 if the point is inside of the polygon
     *
     * REMARKS:
     *      The vertices may be listed clockwise or anticlockwise.
     *      The first may optionally be repeated, if so nvert may optionally be increased by 1.
     *      The input polygon may be a compound polygon constisting of several separate
     *      subpolygons. If so, the first vertex of each subpolygon must be repeated, and when calculating
     *      nvert, these first vertices must be counted twice.
     *      Written by Randolph Franklin, University of Ottawa, 7/70.
     *
     * METHOD:
     *      A vertical line is drawn thru the point in question. If it crosses the polygon an odd number
     *      of times, then the point is inside of the polygon.
     */
    static int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
    {
        int i, j, c = 0;
          for (i = 0, j = nvert-1; i < nvert; j = i++) {
            if ( ((verty[i]>testy) != (verty[j]>testy)) &&
             (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
               c = !c;
          }
          return c;
    }

    /**
     * @brief computePanTiltMatrix given a gaze vector computes the Homogeneous Matrix to control the
     * YAW-PITCH angles.
     * The algorithm used is based on the paper: "Adaptive Predictive Gaze Control of a Redundant Humanoid
     * Robot Head, IROS2011".
     *
     * @param gaze vector [3x1]
     * @param pan_tilt_matrix Homogeneous Matrix [4x4] in the same reference frame of the gaze vector
     */
    static void computePanTiltMatrix(const Eigen::VectorXd& gaze, KDL::Frame& pan_tilt_matrix);

    /**
     * @brief computeCartesianError orientation and position error
     * @param T actual pose Homogeneous Matrix [4x4]
     * @param Td desired pose Homogeneous Matrix [4x4]
     * @param position_error position error [3x1]
     * @param orientation_error orientation error [3x1]
     */
    static void computeCartesianError(const Eigen::Matrix4d &T,
                                      const Eigen::Matrix4d &Td,
                                      Eigen::Vector3d& position_error,
                                      Eigen::Vector3d& orientation_error);
    /**
     * @brief computeCartesianError orientation and position error
     * @param T actual pose
     * @param Td desired pose
     * @param position_error position error
     * @param orientation_error orientation error
     */
    static void computeCartesianError(const Eigen::Affine3d &T,
                                      const Eigen::Affine3d &Td,
                                      Eigen::Vector3d& position_error,
                                      Eigen::Vector3d& orientation_error);

    /**
     * @brief computeGradient compute numerical gradient of a function using 2 points formula:
     *
     *           f(x+h) - f(x-h)
     *   df(x)= ----------------
     *                2h
     * @param x points around gradient is compute
     * @param fun function to derive
     * @param step step of gradient
     * @return vector of gradient
     */
    static Eigen::VectorXd computeGradient(const Eigen::VectorXd &x,
                                              CostFunction &fun,
                                              const double &step = 1E-3);

    /**
     * @brief computeGradient compute numerical gradient of a function using 2 points formula:
     *
     *           f(x+h) - f(x-h)
     *   df(x)= ----------------
     *                2h
     * @param x points around gradient is compute
     * @param fun function to derive
     * @param jointMask the joints over which we want to compute the gradient
     * @param step step of gradient
     * @return vector of gradient
     */
    static Eigen::VectorXd computeGradient(const Eigen::VectorXd &x,
                                              CostFunction &fun,
                                              const std::vector<bool>& jointMask,
                                              const double &step = 1E-3);
};



#endif
