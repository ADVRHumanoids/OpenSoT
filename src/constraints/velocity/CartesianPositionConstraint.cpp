/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;


CartesianPositionConstraint::CartesianPositionConstraint(const Eigen::VectorXd &x,
                                                         OpenSoT::tasks::velocity::Cartesian::Ptr cartesianTask,
                                                         const Eigen::MatrixXd &A_Cartesian,
                                                         const Eigen::VectorXd &b_Cartesian,
                                                         const double boundScaling) :
    Constraint("position_constraint", x.size()),
    _cartesianTask(cartesianTask),
    _A_Cartesian(A_Cartesian),
    _b_Cartesian(b_Cartesian),
    _boundScaling(boundScaling),
    _is_Cartesian(true),
    J(3, x.size()),
    currentPosition(3)
{

    assert(_A_Cartesian.rows() == _b_Cartesian.rows() && "A and b must have the same size");
    assert(_A_Cartesian.cols() == 3 && "A must have 3 columns");

    this->update(x);
}


CartesianPositionConstraint::CartesianPositionConstraint(const Eigen::VectorXd& x,
                             OpenSoT::tasks::velocity::CoM::Ptr comTask,
                             const Eigen::MatrixXd& A_Cartesian,
                             const Eigen::VectorXd& b_Cartesian,
                             const double boundScaling  ):
    Constraint("position_constraint", x.size()),
    _comTask(comTask),
    _A_Cartesian(A_Cartesian),
    _b_Cartesian(b_Cartesian),
    _boundScaling(boundScaling),
    _is_Cartesian(false),
    J(3, x.size()),
    currentPosition(3)
{
    assert(_A_Cartesian.rows() == _b_Cartesian.rows() && "A and b must have the same size");
    assert(_A_Cartesian.cols() == 3 && "A must have 3 columns");

    this->update(x);
}

void CartesianPositionConstraint::setAbCartesian(const Eigen::MatrixXd& A_Cartesian, const Eigen::VectorXd& b_Cartesian)
{
    _A_Cartesian = A_Cartesian;
    _b_Cartesian = b_Cartesian;

    assert(_A_Cartesian.rows() == _b_Cartesian.rows() && "A and b must have the same size");
    assert(_A_Cartesian.cols() == 3 && "A must have 3 columns");
}

void CartesianPositionConstraint::getCurrentPosition(Eigen::VectorXd& current_position)
{
    current_position = currentPosition;
}

void CartesianPositionConstraint::update(const Eigen::VectorXd &x) {

    if(_is_Cartesian){
        _cartesianTask->update(x);
        /************************ COMPUTING BOUNDS ****************************/
        J = _cartesianTask->getA().block(0,0,3,_x_size);
        assert(J.rows() == 3 && "Jacobian doesn't have 3 rows. Something went wrong.");

        _Aineq = _A_Cartesian * J;

        currentPosition(0) = _cartesianTask->getActualPose()(0,3);
        currentPosition(1) = _cartesianTask->getActualPose()(1,3);
        currentPosition(2) = _cartesianTask->getActualPose()(2,3);
        assert(currentPosition.size() == 3 && "Current position doesn't have size 3. Something went wrong.");

        /**********************************************************************/
    }else{
        _comTask->update(x);
        J = _comTask->getA();

        _Aineq = _A_Cartesian * J;

        currentPosition = _comTask->getActualPosition();
    }
    _bUpperBound = ( _b_Cartesian - _A_Cartesian*currentPosition)*_boundScaling;
    _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bUpperBound.size());

}
