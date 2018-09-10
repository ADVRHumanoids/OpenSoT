/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

VelocityLimits::VelocityLimits(const double qDotLimit,
                               const double dT,
                               const unsigned int x_size) :
    Constraint("velocity_limits", x_size), _dT(dT) {

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);
}

VelocityLimits::VelocityLimits(const Eigen::VectorXd& qDotLimit,
                               const double dT) :
    Constraint("velocity_limits", qDotLimit.size()), _dT(dT) {

    _lowerBound.setZero(_x_size);
    _upperBound.setZero(_x_size);

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);
}

Eigen::VectorXd OpenSoT::constraints::velocity::VelocityLimits::getVelocityLimits()
{
    return _upperBound/_dT;
}

void OpenSoT::constraints::velocity::VelocityLimits::setVelocityLimits(const double qDotLimit)
{
    _qDotLimit = std::fabs(qDotLimit);

    this->generateBounds(_qDotLimit);
}

void OpenSoT::constraints::velocity::VelocityLimits::setVelocityLimits(const Eigen::VectorXd& qDotLimit)
{
    this->generateBounds(qDotLimit);
}

double OpenSoT::constraints::velocity::VelocityLimits::getDT()
{
    return _dT;
}

void VelocityLimits::generateBounds(const double qDotLimit)
{
    /************************ COMPUTING BOUNDS ****************************/
        _lowerBound<<_lowerBound.setOnes(_x_size)*-1.0*_qDotLimit*_dT;
        _upperBound<<_upperBound.setOnes(_x_size)*1.0*_qDotLimit*_dT;

    /**********************************************************************/
}

void VelocityLimits::generateBounds(const Eigen::VectorXd& qDotLimit)
{
    assert(qDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < qDotLimit.size(); ++i)
    {
        _lowerBound[i] = -1.0*std::fabs(qDotLimit[i])*_dT;
        _upperBound[i] = 1.0*std::fabs(qDotLimit[i])*_dT;
    }
}
