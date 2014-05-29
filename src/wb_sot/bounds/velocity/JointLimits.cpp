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

using namespace wb_sot::bounds::velocity;

JointLimits::JointLimits(   const iCub::iDynTree::DynTree& robot,
                            const double dT,
                            const unsigned int x_size) :
    Bounds(x_size), _dT(dT), _robot(robot) {
    _x = _robot.getAng();

    /* calling update to generate bounds */
    update(_x);
}

void JointLimits::update(const yarp::sig::Vector& x)
{
    _x = x;

/************************ COMPUTING BOUNDS ****************************/

    _upperBound = (_robot.getJointBoundMax() - _x)*_dT;
    _lowerBound = (_robot.getJointBoundMin() - _x)*_dT;

/**********************************************************************/

}




