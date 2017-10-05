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
                                               const Eigen::MatrixXd &A_Cartesian,
                                               const Eigen::VectorXd &b_Cartesian,
                                               const double dT,
                                               const double boundScaling):
    Constraint("capture_point_constraint", x.size()),
    _dT(dT)
{
    assert(A_Cartesian.rows() == b_Cartesian.rows() && "A and b must have the same size");
    assert(A_Cartesian.cols() == 2 && "A must have 2 columns");

    _A_Cartesian.setZero(A_Cartesian.rows(),3);
    _A_Cartesian.block(0,0,A_Cartesian.rows(),2) = A_Cartesian;

    _b_Cartesian.setZero(b_Cartesian.size());
    _b_Cartesian = b_Cartesian;

    _cartesian_position_cstr.reset(new CartesianPositionConstraint(x, comTask, _A_Cartesian,
                                                                   _b_Cartesian, boundScaling));
    com.setZero(3);
    this->update(x);
}

void CapturePointConstraint::update(const Eigen::VectorXd &x)
{
    _cartesian_position_cstr->update(x);
    _cartesian_position_cstr->getCurrentPosition(com);
    std::cout<<"com: "<<com<<std::endl;

    w = sqrt(fabs(com[2])/9.81)*(1.0/_dT);
    std::cout<<"w: "<<w<<std::endl;

    _Aineq = w*_cartesian_position_cstr->getAineq();
    _bUpperBound = _cartesian_position_cstr->getbUpperBound();
    _bLowerBound = _cartesian_position_cstr->getbLowerBound();
}
