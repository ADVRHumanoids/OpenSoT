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
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

CartesianPositionConstraint::CartesianPositionConstraint(const yarp::sig::Vector &x,
                                                         OpenSoT::tasks::velocity::Cartesian::Ptr cartesianTask,
                                                         const yarp::sig::Matrix &A_Cartesian,
                                                         const yarp::sig::Vector &b_Cartesian,
                                                         const double boundScaling) :
    Constraint(x.size()),
    _cartesianTask(cartesianTask),
    _A_Cartesian(A_Cartesian),
    _b_Cartesian(b_Cartesian),
    _boundScaling(boundScaling)
{
    assert(_A_Cartesian.rows() == _b_Cartesian.size() && "A and b must have the same size");
    assert(_A_Cartesian.cols() == 3 && "A must have 3 columns");

    this->update(x);
}

void CartesianPositionConstraint::update(const yarp::sig::Vector &x) {

    /************************ COMPUTING BOUNDS ****************************/

    yarp::sig::Matrix J = _cartesianTask->getA();
    assert(J.rows() == 6 && "Jacobian doesn't have 6 rows. Is this a cartesian task?");
    J.removeRows(2,3);
    assert(J.rows() == 3 && "Jacobian doesn't have 3 rows. Something went wrong.");

    _Aineq = _A_Cartesian * J;

    yarp::sig::Vector currentPosition = _cartesianTask->getActualPose().getCol(3).subVector(0,2);
    assert(currentPosition.size() == 3 && "Current position doesn't have size 3. Something went wrong.");

    _bUpperBound = ( _b_Cartesian - _A_Cartesian*currentPosition)*_boundScaling;
    /**********************************************************************/
}
