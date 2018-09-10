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

#include <OpenSoT/constraints/velocity/affine/JointLimits.h>

using namespace OpenSoT::constraints::velocity::affine;

JointLimits::JointLimits(   const Eigen::VectorXd& q,
                            const Eigen::VectorXd& jointBoundMax,
                            const Eigen::VectorXd& jointBoundMin,
                            const double boundScaling) :
    Constraint("joint_limits", q.size()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _boundScaling(boundScaling),
    _constr("joint_limits_internal_generic_constr",jointBoundMax, jointBoundMin, q.size())
{

    if(q.size() != _jointLimitsMax.size())
        throw std::runtime_error("q.size() != _jointLimitsMax.size()");
    if(q.size() != _jointLimitsMin.size())
        throw std::runtime_error("q.size() != _jointLimitsMin.size()");
    /* calling update to generate bounds */       

    update(q);
}

JointLimits::JointLimits(const Eigen::VectorXd &q,
            const Eigen::VectorXd &jointBoundMax,
            const Eigen::VectorXd &jointBoundMin,
            const AffineHelper& var,
            const double boundScaling):
    Constraint("joint_limits", var.getInputSize()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _boundScaling(boundScaling),
    _constr("joint_limits_internal_generic_constr",var, jointBoundMax, jointBoundMin, GenericConstraint::Type::CONSTRAINT)
{
    if(q.size() != _jointLimitsMax.size())
        throw std::runtime_error("q.size() != _jointLimitsMax.size()");
    if(q.size() != _jointLimitsMin.size())
        throw std::runtime_error("q.size() != _jointLimitsMin.size()");
    /* calling update to generate bounds */

    update(q);
}


void JointLimits::update(const Eigen::VectorXd& x)
{

/************************ COMPUTING BOUNDS ****************************/

    __upperBound = ( _jointLimitsMax - x)*_boundScaling;
    __lowerBound = ( _jointLimitsMin - x)*_boundScaling;
    
    __upperBound = __upperBound.cwiseMax(0.0);
    __lowerBound = __lowerBound.cwiseMin(0.0);

/**********************************************************************/

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

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}




