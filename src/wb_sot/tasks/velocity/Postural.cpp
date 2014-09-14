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

#include <wb_sot/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace wb_sot::tasks::velocity;
using namespace yarp::math;

Postural::Postural(   const yarp::sig::Vector& x) :
    Task("postural", x.size()), _x(x)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    _hessianType = HST_POSDEF;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->update(x);
}

Postural::~Postural()
{
}

void Postural::update(const yarp::sig::Vector &x) {
    _x = x;

    /************************* COMPUTING TASK *****************************/
    this->update_b();
    /**********************************************************************/
}

void Postural::setReference(const yarp::sig::Vector& x_desired) {
    _x_desired = x_desired;
    this->update_b();
}

void Postural::update_b() {
    _b = _x_desired - _x;
}



