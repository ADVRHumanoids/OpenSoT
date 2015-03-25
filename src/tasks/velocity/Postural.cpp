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

#include <OpenSoT/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

Postural::Postural(   const yarp::sig::Vector& x) :
    Task("Postural", x.size()), _x(x),
    _x_desired(x.size(),0.0), _xdot_desired(x.size(),0.0)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->_update(x);
}

Postural::~Postural()
{
}

void Postural::_update(const yarp::sig::Vector &x) {
    _x = x;

    /************************* COMPUTING TASK *****************************/

    this->update_b();

    _xdot_desired.zero();

    /**********************************************************************/
}

void Postural::setReference(const yarp::sig::Vector& x_desired) {
    assert(x_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired.zero();
    this->update_b();
}

void OpenSoT::tasks::velocity::Postural::setReference(const yarp::sig::Vector &x_desired,
                                                      const yarp::sig::Vector &xdot_desired)
{
    assert(x_desired.size() == _x_size);
    assert(xdot_desired.size() == _x_size);

    _x_desired = x_desired;
    _xdot_desired = xdot_desired;
    this->update_b();
}

yarp::sig::Vector OpenSoT::tasks::velocity::Postural::getReference() const
{
    return _x_desired;
}

void OpenSoT::tasks::velocity::Postural::getReference(yarp::sig::Vector &x_desired,
                                                      yarp::sig::Vector &xdot_desired) const
{
    x_desired = _x_desired;
    xdot_desired = _xdot_desired;
}

void Postural::update_b() {
    if(_lambda < 1E-9)
        _b = (_x_desired - _x);
    else
        _b = (_x_desired - _x) + _xdot_desired/_lambda;
}

void OpenSoT::tasks::velocity::Postural::setLambda(double lambda)
{
    this->_lambda = lambda;
    this->update_b();
}

yarp::sig::Vector OpenSoT::tasks::velocity::Postural::getActualPositions()
{
    return _x;
}



