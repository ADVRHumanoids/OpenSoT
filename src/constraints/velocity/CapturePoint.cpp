/*
 * Copyright (C) 2017 Cogimon
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OpenSoT/constraints/velocity/CapturePoint.h>

using namespace OpenSoT::constraints::velocity;

CapturePointConstraint::CapturePointConstraint(const Eigen::VectorXd &x,
                                               OpenSoT::tasks::velocity::CoM::Ptr comTask,
                                               XBot::ModelInterface& robot,
                                               const Eigen::MatrixXd &A_Cartesian,
                                               const Eigen::VectorXd &b_Cartesian,
                                               const double dT,
                                               const double boundScaling):
    Constraint("capture_point_constraint", x.size()),
    _dT(dT),
    _robot(robot)
{
    assert(A_Cartesian.rows() == b_Cartesian.rows() && "A and b must have the same size");
    assert(A_Cartesian.cols() == 2 && "A must have 2 columns");

    _A_Cartesian.setZero(A_Cartesian.rows(),3);
    _A_Cartesian.block(0,0,A_Cartesian.rows(),2) = A_Cartesian;

    _b_Cartesian.setZero(b_Cartesian.size());
    _b_Cartesian = b_Cartesian;

    _cartesian_position_cstr.reset(new CartesianPositionConstraint(x, comTask, _A_Cartesian,
                                                                   _b_Cartesian, boundScaling));
    w = 1.0;
    com.setZero(3);

    _H.resize(6, _x_size);
    _H2.resize(2, _x_size);
    this->update(x);
}

void CapturePointConstraint::update(const Eigen::VectorXd &x)
{
    _cartesian_position_cstr->update(x);
    _cartesian_position_cstr->getCurrentPosition(com);

    w = sqrt(fabs(com[2])/9.81)*(1.0/_dT);

    _robot.getCentroidalMomentumMatrix(_H);

    _H = (w/(_robot.getMass()*com[2]))*_H;
    _H2<<_H.block(4,0,1,_x_size),
         -_H.block(3,0,1,_x_size);

    _Aineq = w*_cartesian_position_cstr->getAineq() + _A_Cartesian.block(0,0,_A_Cartesian.rows(),2)*_H2;
    _bUpperBound = _cartesian_position_cstr->getbUpperBound();
    _bLowerBound = _cartesian_position_cstr->getbLowerBound();
}

void CapturePointConstraint::setAbCartesian(const Eigen::MatrixXd& A_Cartesian, const Eigen::VectorXd& b_Cartesian)
{
    assert(A_Cartesian.rows() == b_Cartesian.rows() && "A and b must have the same size");
    assert(A_Cartesian.cols() == 2 && "A must have 2 columns");

    _A_Cartesian.setZero(A_Cartesian.rows(),3);
    _A_Cartesian.block(0,0,A_Cartesian.rows(),2) = A_Cartesian;

    _b_Cartesian.setZero(b_Cartesian.size());
    _b_Cartesian = b_Cartesian;

    _cartesian_position_cstr->setAbCartesian(_A_Cartesian, _b_Cartesian);
}
