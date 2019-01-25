/*
 * Copyright (C) 2019 Cogimon
 * Author: Matteo Parigi Polverini
 * email:  matteo.parigi@iit.it
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

#include <OpenSoT/constraints/acceleration/JointLimits.h>
#include <boost/make_shared.hpp>

using namespace OpenSoT::constraints::acceleration;

JointLimits::JointLimits(   XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointAccMax):
     Constraint("joint_limits", qddot.getInputSize()),
     _jointLimitsMax(jointBoundMax),
     _jointLimitsMin(jointBoundMin),
     _jointAccMax(jointAccMax),
     _robot(robot)
{


    if(qddot.getOutputSize() != _jointLimitsMax.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMax.size()");
    if(qddot.getOutputSize() != _jointLimitsMin.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMin.size()");
    if(_jointAccMax.size() != _jointLimitsMin.size())
        throw std::runtime_error("_jointAccMax.size() != _jointAccMax.size()");
    /* calling update to generate bounds */       
    
    
    _generic_constraint_internal = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                _jointAccMax,
                -_jointAccMax,
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    update(Eigen::VectorXd(1));
}


void JointLimits::update(const Eigen::VectorXd& x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    __upperBound =  _jointAccMax;
    __lowerBound = -_jointAccMax;

    _invFunLowerBound.setOnes(_q.size()); _invFunLowerBound *= -1.;
    _invFunUpperBound.setOnes(_q.size()); _invFunUpperBound *= -1.;
        
    for(int i = 0; i < _q.size(); i++)
    {            
        if (_qdot(i) == 0)
        {
            _invFunUpperBound(i) =  _q(i) - _jointLimitsMax(i);
            _invFunLowerBound(i) = -_q(i) + _jointLimitsMin(i);
        }
        else if (_qdot(i) < 0)
        {
            _invFunUpperBound(i) =  _q(i) - _jointLimitsMax(i);
            _invFunLowerBound(i) = -_q(i) + _jointLimitsMin(i) - 1/(2*_jointAccMax(i))*pow(_qdot(i),2);
        }
        else
        {
             _invFunUpperBound(i) =  x(i) - _jointLimitsMax(i) + 1/(2*_jointAccMax(i))*pow(_qdot(i),2);
             _invFunLowerBound(i) = -x(i) + _jointLimitsMin(i);
        }
             
        if (_invFunUpperBound(i) >= 0)
        {
            __upperBound(i) = - _jointAccMax(i);
            __lowerBound(i) = - _jointAccMax(i);
        }
             
        if (_invFunLowerBound(i) >= 0)
        {
            __upperBound(i) =  _jointAccMax(i);
            __lowerBound(i) =  _jointAccMax(i);
        }
                         
    }
    
     _generic_constraint_internal->setBounds(__upperBound, __lowerBound);
     
     _generic_constraint_internal->update(x);
     
     _Aineq = _generic_constraint_internal->getAineq();
     _bLowerBound = _generic_constraint_internal->getbLowerBound();
     _bUpperBound = _generic_constraint_internal->getbUpperBound();   
}


