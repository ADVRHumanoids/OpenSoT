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

#include <wb_sot/constraints/velocity/VelocityLimits.h>
#include <yarp/math/Math.h>
#include <cmath>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

VelocityLimits::VelocityLimits(const double qDotLimit,
                               const double dT,
                               const unsigned int x_size) :
    Constraint(x_size), _dT(dT) {

    _qDotLimit = std::abs(qDotLimit)*_dT;
/************************ COMPUTING BOUNDS ****************************/

    _lowerBound.resize(x_size,-1.0*_qDotLimit);
    _upperBound.resize(x_size,+1.0*_qDotLimit);

/**********************************************************************/

}


