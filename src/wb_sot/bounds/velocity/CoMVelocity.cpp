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

#include <wb_sot/bounds/velocity/CoMVelocity.h>
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace wb_sot::bounds::velocity;
using namespace yarp::math;

CoMVelocity::CoMVelocity(const yarp::sig::Vector velocityLimits,
                         const iDynUtils &robot,
                         const double dT,
                         const unsigned int x_size) :
Bounds(x_size), _robot(robot),_dT(dT), _velocityLimits(velocityLimits) {

    if(_velocityLimits.size() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";

    _Aineq.resize(3, x_size);
    _bLowerBound.resize(x_size);
    _bUpperBound.resize(x_size);
    this->_support_foot_linkIndex = _robot.left_leg.index;
    _robot.coman_iDyn3.setFloatingBaseLink(_support_foot_linkIndex);
    this->update(yarp::sig::Vector(x_size, 0.0));

    /******************** COMPUTING CONSTANT BOUNDS ***********************/

    _bLowerBound = -1.0*_velocityLimits*_dT;
    _bUpperBound = +1.0*_velocityLimits*_dT;

    /**********************************************************************/
}

void CoMVelocity::update(const yarp::sig::Vector &x) {
    yarp::sig::Matrix JCoM;
    _robot.updateiDyn3Model(x);

    if(!_robot.coman_iDyn3.getCOMJacobian(JCoM))
        throw "Error computing CoM Jacobian";

    JCoM = JCoM.removeCols(0,6);    // remove floating base
    JCoM = JCoM.removeRows(3,3);    // remove orientation

    /************************ COMPUTING BOUNDS ****************************/

    _Aineq = JCoM;

    /**********************************************************************/
}


