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
                            const unsigned int x_size,
                            const double boundScaling) :
    Bounds(x_size), _robot(robot), _boundScaling(boundScaling) {

    /* calling update to generate bounds */
    update(_robot.getAng());
}

void JointLimits::update(const yarp::sig::Vector& x)
{

/************************ COMPUTING BOUNDS ****************************/

    _upperBound = (_robot.getJointBoundMax() - x)*_boundScaling;
    _lowerBound = (_robot.getJointBoundMin() - x)*_boundScaling;

/**********************************************************************/

}




