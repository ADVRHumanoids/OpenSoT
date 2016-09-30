/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo
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

#include <OpenSoT/constraints/virtual_model/TorqueLimits.h>
#include <yarp/math/Math.h>
#include <cmath>

using namespace OpenSoT::constraints::virtual_model;
using namespace yarp::math;

TorqueLimits::TorqueLimits(const yarp::sig::Vector& torque_limits) :
    Constraint("torque_limits", torque_limits.size()){

    _upperBound = torque_limits;
    _lowerBound = -1.*torque_limits;

}

void TorqueLimits::getTorqueLimits(yarp::sig::Vector& torque_limits)
{
    torque_limits = _upperBound;
}

void TorqueLimits::setTorqueLimits(const yarp::sig::Vector& torque_limits)
{
    _upperBound = torque_limits;
    _lowerBound = -1.*torque_limits;
}


