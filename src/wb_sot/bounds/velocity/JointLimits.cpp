/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#include <wb_sot/bounds/velocity/JointLimits.h>
#include <yarp/math/Math.h>
using namespace yarp::math;

using namespace wb_sot::bounds;

template <unsigned int x_size>
JointLimits<x_size>::JointLimits(const iCub::iDynTree::DynTree& robot,
                                 const double dT) :
    _dT(dT), _robot(robot) {
    _q = _robot.getAng();

    /* calling update to generate bounds */
    update(_q);
}

template <unsigned int x_size>
yarp::sig::Vector JointLimits<x_size>::getLowerBound() {
    return _qLowerBounds;
}

template <unsigned int x_size>
yarp::sig::Vector JointLimits<x_size>::getUpperBound() {
    return _qLowerBounds;
}

template <unsigned int x_size>
void JointLimits<x_size>::update(const yarp::sig::Vector& x)
{
    _q = x;
    /* actually computing bounds... */
    _qUpperBounds = (_robot.getJointBoundMax() - _q)*_dT;
    _qLowerBounds = (_robot.getJointBoundMin() - _q)*_dT;
}





/*************** RETURN ZERO DIMENSION MATRICES/VECTORS **************/
template <unsigned int x_size>
yarp::sig::Matrix  JointLimits<x_size>::getAeq() {
    return yarp::sig::Matrix(0,0);
}

template <unsigned int x_size>
yarp::sig::Vector JointLimits<x_size>::getbeq() {
    return yarp::sig::Vector(0);
}

template <unsigned int x_size>
yarp::sig::Matrix JointLimits<x_size>::getAineq() {
    return yarp::sig::Matrix(0,0);
}

template <unsigned int x_size>
yarp::sig::Vector JointLimits<x_size>::getbLowerBound() {
    return yarp::sig::Vector(0);
}

template <unsigned int x_size>
yarp::sig::Vector JointLimits<x_size>::getbUpperBound() {
    return yarp::sig::Vector(0);
}

