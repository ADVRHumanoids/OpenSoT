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

#include <OpenSoT/constraints/velocity/affine/VelocityLimits.h>
#include <cmath>

using namespace OpenSoT::constraints::velocity::affine;

VelocityLimits::VelocityLimits(const double qDotLimit,
               const double dT,
               const AffineHelper& var):
    Constraint("velocity_limits", var.getInputSize()), _dT(dT),
    _constr("velocity_limits_internal_generic_constr", var,
            Eigen::VectorXd::Constant(var.getOutputSize(), qDotLimit*dT),
            Eigen::VectorXd::Constant(var.getOutputSize(), -qDotLimit*dT),
            GenericConstraint::Type::CONSTRAINT)
{
    __lowerBound.setZero(var.getOutputSize());
    __upperBound.setZero(var.getOutputSize());

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);

    this->update(Eigen::VectorXd(1));
}

VelocityLimits::VelocityLimits(const Eigen::VectorXd& qDotLimit,
               const double dT,
               const AffineHelper& var):
    Constraint("velocity_limits", var.getInputSize()), _dT(dT),
    _constr("velocity_limits_internal_generic_constr", var,
            qDotLimit*dT, -qDotLimit*dT, GenericConstraint::Type::CONSTRAINT)
{
    __lowerBound.setZero(qDotLimit.size());
    __upperBound.setZero(qDotLimit.size());

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);

    this->update(Eigen::VectorXd(1));
}

VelocityLimits::VelocityLimits(const double qDotLimit,
                               const double dT,
                               const unsigned int x_size) :
    Constraint("velocity_limits", x_size), _dT(dT),
    _constr("velocity_limits_internal_generic_constr",
            Eigen::VectorXd::Constant(x_size, qDotLimit*dT),
            Eigen::VectorXd::Constant(x_size, -qDotLimit*dT), x_size)
{

    __lowerBound.setZero(x_size);
    __upperBound.setZero(x_size);

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);

    this->update(Eigen::VectorXd(1));
}

VelocityLimits::VelocityLimits(const Eigen::VectorXd& qDotLimit,
                               const double dT) :
    Constraint("velocity_limits", qDotLimit.size()), _dT(dT),
    _constr("velocity_limits_internal_generic_constr", qDotLimit*dT, -qDotLimit*dT, qDotLimit.size())
{

    __lowerBound.setZero(qDotLimit.size());
    __upperBound.setZero(qDotLimit.size());

   this->setVelocityLimits(qDotLimit);

    this->generateBounds(qDotLimit);

    this->update(Eigen::VectorXd(1));
}

Eigen::VectorXd VelocityLimits::getVelocityLimits()
{
    return __upperBound/_dT;
}

void VelocityLimits::setVelocityLimits(const double qDotLimit)
{
    _qDotLimit = std::fabs(qDotLimit);

    this->generateBounds(_qDotLimit);
}

void VelocityLimits::setVelocityLimits(const Eigen::VectorXd& qDotLimit)
{
    this->generateBounds(qDotLimit);
}

double VelocityLimits::getDT()
{
    return _dT;
}

void VelocityLimits::generateBounds(const double qDotLimit)
{
    /************************ COMPUTING BOUNDS ****************************/
        __lowerBound<<__lowerBound.setOnes(__lowerBound.size())*-1.0*_qDotLimit*_dT;
        __upperBound<<__upperBound.setOnes(__upperBound.size())*1.0*_qDotLimit*_dT;

    /**********************************************************************/
}

void VelocityLimits::generateBounds(const Eigen::VectorXd& qDotLimit)
{
    assert(qDotLimit.size() == _x_size);
    for(unsigned int i = 0; i < qDotLimit.size(); ++i)
    {
        __lowerBound[i] = -1.0*std::fabs(qDotLimit[i])*_dT;
        __upperBound[i] = 1.0*std::fabs(qDotLimit[i])*_dT;
    }
}

void VelocityLimits::update(const Eigen::VectorXd& x)
{
    if(!_constr.setBounds(__upperBound, __lowerBound))
        XBot::Logger::error("%s: error in _constr.setBounds(__upperBound, __lowerBound)!", getConstraintID());

    if(_constr.getType() == GenericConstraint::Type::BOUND)
    {
        _upperBound = _constr.getUpperBound();
        _lowerBound = _constr.getLowerBound();
    }
    else if(_constr.getType() == GenericConstraint::Type::CONSTRAINT)
    {
        _Aineq = _constr.getAineq();
        _bUpperBound = _constr.getbUpperBound();
        _bLowerBound = _constr.getbLowerBound();
    }

}
