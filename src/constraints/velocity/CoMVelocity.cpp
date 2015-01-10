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

#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

CoMVelocity::CoMVelocity(const yarp::sig::Vector velocityLimits,
                         const double dT,
                         const yarp::sig::Vector& x,
                         iDynUtils &robot) :
Constraint(x.size()), _dT(dT), _velocityLimits(velocityLimits),
_robot(robot) {

    if(_velocityLimits.size() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";

    _Aineq.resize(3, x.size());
    _bLowerBound.resize(x.size());
    _bUpperBound.resize(x.size());

    this->generatebBounds();

    this->update(x);
}

void CoMVelocity::update(const yarp::sig::Vector &x) {

    yarp::sig::Matrix JCoM;
    if(!_robot.iDyn3_model.getCOMJacobian(JCoM))
        throw "Error computing CoM Jacobian";
    JCoM.removeRows(3,3);       // remove orientation
    JCoM.removeCols(0,6);       // remove floating base

    /************************ COMPUTING BOUNDS ****************************/

    _Aineq = JCoM;

    /**********************************************************************/
}

yarp::sig::Vector OpenSoT::constraints::velocity::CoMVelocity::getVelocityLimits()
{
    return _velocityLimits;
}

void OpenSoT::constraints::velocity::CoMVelocity::setVelocityLimits(const yarp::sig::Vector velocityLimits)
{
    if(_velocityLimits.size() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";
    _velocityLimits = velocityLimits;
}

void CoMVelocity::generatebBounds() {

    /******************** COMPUTING CONSTANT BOUNDS ***********************/

    _bLowerBound = -1.0*_velocityLimits*_dT;
    _bUpperBound = +1.0*_velocityLimits*_dT;

    /**********************************************************************/
}


