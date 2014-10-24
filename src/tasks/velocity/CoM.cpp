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

#include <OpenSoT/tasks/velocity/CoM.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

CoM::CoM(   const yarp::sig::Vector& x,
            iDynUtils &robot) :
    Task("com", x.size()), _robot(robot),
    _desiredPosition(3,0.0), _actualPosition(3,0.0),
    positionError(3, 0.0)
{

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);


    /* initializing to zero error */
    _desiredPosition = _actualPosition;
    _b.zero();
    positionError = _b;

    _W.resize(3,3);
    _W.eye();

    _hessianType = HST_SEMIDEF;
}

CoM::~CoM()
{
}

void CoM::_update(const yarp::sig::Vector &x) {

    /************************* COMPUTING TASK *****************************/

    _actualPosition = _robot.coman_iDyn3.getCOM();

    //This part of code is an HACK due to a bug in iDynTree
    assert(_robot.coman_iDyn3.getCOMJacobian(_A));

    _A = _A.removeCols(0,6);    // remove floating base
    _A = _A.removeRows(3,3);    // remove orientation

    this->update_b();
    /**********************************************************************/
}

void CoM::setReference(const yarp::sig::Vector& desiredPosition) {
    _desiredPosition = desiredPosition;
    this->update_b();
}

yarp::sig::Vector CoM::getReference() {
    return _desiredPosition;
}

yarp::sig::Vector CoM::getActualPosition() {
    return _actualPosition;
}

void CoM::update_b() {
    _b = _desiredPosition - _actualPosition;
    positionError = _b;
}
