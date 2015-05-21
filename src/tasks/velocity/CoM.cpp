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
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>
#include <iCub/iDynTree/yarp_kdl.h>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

#define LAMBDA_THS 1E-12

CoM::CoM(   const yarp::sig::Vector& x,
            iDynUtils &robot) :
    Task("CoM", x.size()), _robot(robot),
    _desiredPosition(3,0.0), _actualPosition(3,0.0),
    positionError(3, 0.0), _desiredVelocity(3,0.0)
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

void CoM::_update(const yarp::sig::Vector &x)
{

    /************************* COMPUTING TASK *****************************/

    _actualPosition = _robot.iDyn3_model.getCOM();

    bool res = _robot.iDyn3_model.getCOMJacobian(_A);
    assert(res);
    _A = _A.removeRows(3,3);    // remove orientation
    _A = _A.removeCols(0,6);    // remove floating base

    this->update_b();

    this->_desiredVelocity.zero();

    /**********************************************************************/
}

void CoM::setReference(const yarp::sig::Vector& desiredPosition)
{
    assert(desiredPosition.size() == 3);

	_desiredPosition = desiredPosition;
    _desiredVelocity.zero();
    this->update_b();
}

void OpenSoT::tasks::velocity::CoM::setReference(const yarp::sig::Vector &desiredPosition,
                                                 const yarp::sig::Vector &desiredVelocity)
{
    assert(desiredPosition.size() == 3);
    assert(desiredVelocity.size() == 3);

    _desiredPosition = desiredPosition;
    _desiredVelocity = desiredVelocity;
    this->update_b();
}

yarp::sig::Vector CoM::getReference() const
{
    return _desiredPosition;
}

void OpenSoT::tasks::velocity::CoM::getReference(yarp::sig::Vector &desiredPosition, yarp::sig::Vector &desiredVelocity) const
{
    desiredPosition = _desiredPosition;
    desiredVelocity = _desiredVelocity;
}

yarp::sig::Vector CoM::getActualPosition() const
{
    return _actualPosition;
}

std::string OpenSoT::tasks::velocity::CoM::getBaseLink()
{
    return BASE_LINK_COM;
}

std::string OpenSoT::tasks::velocity::CoM::getDistalLink()
{
    return DISTAL_LINK_COM;
}

void CoM::update_b()
{
    _b = _desiredVelocity + _lambda*this->getError();
}

void OpenSoT::tasks::velocity::CoM::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

yarp::sig::Vector OpenSoT::tasks::velocity::CoM::getError()
{
    return _desiredPosition - _actualPosition;
}
