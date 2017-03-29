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

#include <OpenSoT/tasks/velocity/MinimumVelocity.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

MinimumVelocity::MinimumVelocity(const int& x_size) :
    Task("MinimumVelocity", x_size)
{
    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _b.resize(_x_size);
    _b.setZero(_x_size);

    _hessianType = HST_IDENTITY;
}

MinimumVelocity::~MinimumVelocity()
{
}

void MinimumVelocity::_update(const Eigen::VectorXd &x)
{
}

// TODO setWeight should not really change the weight - and same getWeight




