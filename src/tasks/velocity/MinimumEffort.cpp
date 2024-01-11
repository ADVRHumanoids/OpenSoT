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


MinimumEffort::MinimumEffort(   const Eigen::VectorXd& x, const XBot::ModelInterface& robot_model) :
    Task("min_effort", robot_model.getNv()), _gTauGradientWorker(x, robot_model), _x(x)
{
    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);
}

MinimumEffort::~MinimumEffort()
{

}

void MinimumEffort::_update(const Eigen::VectorXd &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    _b = -1.0 * _lambda * cartesian_utils::computeGradient(x, _gTauGradientWorker, this->getActiveJointsMask());

    /**********************************************************************/
}

double MinimumEffort::computeEffort()
{
    return _gTauGradientWorker.compute(_x);
}



