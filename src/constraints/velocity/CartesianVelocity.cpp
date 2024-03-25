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

CartesianVelocity::CartesianVelocity(const Eigen::Vector6d velocityLimits,
                         			 const double dT,
                                     const OpenSoT::tasks::velocity::Cartesian::Ptr& task) :
Constraint(task->getTaskID()+"_VelocityLimit", task->getXSize()), _dT(dT), _velocityLimits(velocityLimits),
_cartesian_task(task), _is_cartesian(true) {

    _Aineq.resize(6,    task->getXSize());
    _bLowerBound.resize(3);
    _bUpperBound.resize(3);

    this->generatebBounds();

    this->generateAineq();
}

CartesianVelocity::CartesianVelocity(const Eigen::Vector3d velocityLimits,
                  const double dT,
                  const OpenSoT::tasks::velocity::CoM::Ptr &task) :
Constraint(task->getTaskID()+"_VelocityLimit", task->getXSize()), _dT(dT), _velocityLimits(velocityLimits),
_com_task(task), _is_cartesian(false)
{
    _Aineq.resize(3,    task->getXSize());
    _bLowerBound.resize(3);
    _bUpperBound.resize(3);

    this->generatebBounds();

    this->generateAineq();
}



void CartesianVelocity::update() {

    this->generateAineq();

}

const Eigen::VectorXd &OpenSoT::constraints::velocity::CartesianVelocity::getVelocityLimits() const
{
    return _velocityLimits;
}

void OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(const Eigen::Vector6d &velocityLimits)
{
    _velocityLimits = velocityLimits;
}

void OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(const Eigen::Vector3d &velocityLimits)
{
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
    if(_is_cartesian)
        _Aineq = _cartesian_task->getA();
    else
        _Aineq = _com_task->getA();

    /**********************************************************************/
}

