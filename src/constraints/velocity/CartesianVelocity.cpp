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

#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

CartesianVelocity::CartesianVelocity(const Eigen::VectorXd velocityLimits,
                         			 const double dT,
                                     OpenSoT::tasks::velocity::Cartesian::Ptr& task) :
Constraint(task->getTaskID()+"_VelocityLimit", task->getXSize()), _dT(dT), _velocityLimits(velocityLimits),
_task(task) {

    if(_velocityLimits.rows() < 6 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";

    _Aineq.resize(6,    task->getXSize());
    _bLowerBound.resize(task->getXSize());
    _bUpperBound.resize(task->getXSize());

    this->generatebBounds();

    this->generateAineq();
}

void CartesianVelocity::update(const Eigen::VectorXd &x) {

    this->generateAineq();

}

Eigen::VectorXd OpenSoT::constraints::velocity::CartesianVelocity::getVelocityLimits()
{
    return _velocityLimits;
}

void OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(const Eigen::VectorXd velocityLimits)
{
    if(_velocityLimits.rows() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";
    _velocityLimits = velocityLimits;
}

void CartesianVelocity::generatebBounds() {

    /******************** COMPUTING CONSTANT BOUNDS ***********************/

    _bLowerBound = -1.0*_velocityLimits*_dT;
    _bUpperBound = +1.0*_velocityLimits*_dT;

    /**********************************************************************/
}

void CartesianVelocity::generateAineq()
{
    /************************ COMPUTING BOUNDS ****************************/

	_Aineq = _task->getA();

    /**********************************************************************/
}

