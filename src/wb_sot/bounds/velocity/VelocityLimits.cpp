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

#include <wb_sot/bounds/velocity/VelocityLimits.h>
#include <yarp/math/Math.h>
#include <cmath>

using namespace wb_sot::bounds;
using namespace yarp::math;

template <unsigned int x_size>
VelocityLimits<x_size>::VelocityLimits(const double qDotLimit, const double dT) :
    _dT(dT) {

    _qDotLimit = std::abs(qDotLimit);

    /* bounds are constant, computing them once for all to generate bounds */
    _qLowerBound = -1*_qDotLimit*yarp::sig::Vector(x_size,1.0);
    _qUpperBound = +1*_qDotLimit*yarp::sig::Vector(x_size,1.0);
}

template <unsigned int x_size>
yarp::sig::Vector VelocityLimits<x_size>::getLowerBound() {
    return _qLowerBound;
}

template <unsigned int x_size>
yarp::sig::Vector VelocityLimits<x_size>::getUpperBound() {
    return _qUpperBound;
}

/*************** RETURN ZERO DIMENSION MATRICES/VECTORS **************/
template <unsigned int x_size>
yarp::sig::Matrix  VelocityLimits<x_size>::getAeq() {
    return yarp::sig::Matrix(0,0);
}

template <unsigned int x_size>
yarp::sig::Vector VelocityLimits<x_size>::getbeq() {
    return yarp::sig::Vector(0);
}

template <unsigned int x_size>
yarp::sig::Matrix VelocityLimits<x_size>::getAineq() {
    return yarp::sig::Matrix(0,0);
}

template <unsigned int x_size>
yarp::sig::Vector VelocityLimits<x_size>::getbLowerBound() {
    return yarp::sig::Vector(0);
}

template <unsigned int x_size>
yarp::sig::Vector VelocityLimits<x_size>::getbUpperBound() {
    return yarp::sig::Vector(0);
}

