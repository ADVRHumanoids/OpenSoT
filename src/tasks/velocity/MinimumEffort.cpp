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

#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

MinimumEffort::MinimumEffort(   const yarp::sig::Vector& x, const iDynUtils& robot_model) :
    Task("min_effort", x.size()), _gTauGradientWorker(x, robot_model), _x(x)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.eye();

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);
}

MinimumEffort::~MinimumEffort()
{

}

void MinimumEffort::_update(const yarp::sig::Vector &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    _b = -1.0 * cartesian_utils::computeGradient(x, _gTauGradientWorker);

    /**********************************************************************/
}

double MinimumEffort::computeEffort()
{
    return _gTauGradientWorker.compute(_x);
}



