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

#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <yarp/math/Math.h>
using namespace yarp::math;

using namespace OpenSoT::constraints::velocity;

JointLimits::JointLimits(   const yarp::sig::Vector& q,
                            const yarp::sig::Vector& jointBoundMax,
                            const yarp::sig::Vector& jointBoundMin,
                            const double boundScaling) :
    Constraint(q.size()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _boundScaling(boundScaling) {

    assert(q.size() == _jointLimitsMax.size());
    assert(q.size() == _jointLimitsMin.size());
    /* calling update to generate bounds */
    update(q);
}

void JointLimits::update(const yarp::sig::Vector& x)
{

/************************ COMPUTING BOUNDS ****************************/

    _upperBound = ( _jointLimitsMax - x)*_boundScaling;
    _lowerBound = ( _jointLimitsMin - x)*_boundScaling;

/**********************************************************************/

}

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}




