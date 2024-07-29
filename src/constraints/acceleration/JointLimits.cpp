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
#include <memory>

using namespace OpenSoT::constraints::acceleration;

JointLimits::JointLimits(   XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointAccMax,
                            const double dt):
     Constraint("joint_limits", qddot.getInputSize()),
     _jointLimitsMax(jointBoundMax),
     _jointLimitsMin(jointBoundMin),
     _jointAccMax(jointAccMax),
     _robot(robot),
     _dt(dt),
     _p(1.0)
{
    _zeros = _robot.getNeutralQ();

    if(qddot.getOutputSize() != _jointLimitsMax.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMax.size()");
    if(qddot.getOutputSize() != _jointLimitsMin.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMin.size()");
    if(_jointAccMax.size() != _jointLimitsMin.size())
        throw std::runtime_error("_jointAccMax.size() != _jointLimitsMin.size()");
    /* calling update to generate bounds */       
    
    
    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                _jointAccMax,
                -_jointAccMax,
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    update();
}


void JointLimits::update()
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);
    
    __upperBound =  _jointAccMax;
    __lowerBound = -_jointAccMax;

    _a.setZero(_robot.getNv());
    _b_sup.setZero(_robot.getNv()); _b_inf.setZero(_robot.getNv());
    _c_sup.setZero(_robot.getNv()); _c_inf.setZero(_robot.getNv());
    _delta_sup.setZero(_robot.getNv()); _delta_inf.setZero(_robot.getNv());
    _ub_sup.setZero(_robot.getNv()); _ub_inf.setZero(_robot.getNv());
    _lb_sup.setZero(_robot.getNv()); _lb_inf.setZero(_robot.getNv());
    _ub.setZero(_robot.getNv()); _lb.setZero(_robot.getNv());
    
    double dt = _dt * _p;

    _a = .5*dt*dt*_jointAccMax.array().cwiseInverse();

    // MAX JOINT LIMITS
    _b_sup = dt*_qdot.array()*_jointAccMax.array().cwiseInverse() + .5*dt*dt;
    _c_sup = _robot.difference(_q, _zeros).array()
         + dt*_qdot.array() - _jointLimitsMax.array() + .5*_qdot.array()*_qdot.array()*_jointAccMax.array().cwiseInverse();
    _delta_sup = _b_sup.array()*_b_sup.array() -4*_a.array()*_c_sup.array();


    // MIN JOINT LIMITS
    _b_inf = dt*_qdot.array()*_jointAccMax.array().cwiseInverse() - .5*dt*dt;
    _c_inf = _robot.difference(-_q, _zeros).array()
        -dt*_qdot.array() + _jointLimitsMin.array() + .5*_qdot.array()*_qdot.array()*_jointAccMax.array().cwiseInverse();
    _delta_inf = _b_inf.array()*_b_inf.array() -4*_a.array()*_c_inf.array();


    for(unsigned int i = 0; i < _delta_inf.size(); ++i)
    {
        if (_delta_sup(i) < 0)
            _delta_sup(i) = 0;
        _ub_sup[i] = .5/_a[i]*(-_b_sup[i] + std::sqrt(_delta_sup[i]));
        _lb_sup[i] = .5/_a[i]*(-_b_sup[i] - std::sqrt(_delta_sup[i]));


        if (_delta_inf(i) < 0)
            _delta_inf(i) = 0;
        _ub_inf[i] = .5/_a[i]*(-_b_inf[i] + std::sqrt(_delta_inf[i]));
        _lb_inf[i] = .5/_a[i]*(-_b_inf[i] - std::sqrt(_delta_inf[i]));


        _ub(i) = std::min(_ub_sup(i),_ub_inf(i));
        _lb(i) = std::max(_lb_sup(i),_lb_inf(i));

        if (_ub(i) < _lb(i))
        {
            _ub(i) = _lb(i);
        }

        __upperBound(i) =  _ub(i);
        __lowerBound(i) =  _lb(i);
    }


    //OLD IMPLEMENTATION
//    for(int i = 0; i < _q.size(); i++)
//    {
   
//        _a(i) = .5*dt*dt/_jointAccMax(i);
        
//        // MAX JOINT LIMITS
//        _b_sup(i) = dt*_qdot(i)/_jointAccMax(i) + .5*dt*dt;
//        _c_sup(i) = _q(i) + dt*_qdot(i) - _jointLimitsMax(i) + .5*_qdot(i)*_qdot(i)/_jointAccMax(i);
        
//        _delta_sup(i) = _b_sup(i)*_b_sup(i) - 4*_a(i)*_c_sup(i);
        
//        if (_delta_sup(i) < 0)
//        {
//            _delta_sup(i) = 0;
//        }
        
//        _ub_sup(i) = .5/_a(i)*(-_b_sup(i) + std::sqrt(_delta_sup(i)));
//        _lb_sup(i) = .5/_a(i)*(-_b_sup(i) - std::sqrt(_delta_sup(i)));
        
//        // MIN JOINT LIMITS
//        _b_inf(i) = dt*_qdot(i)/_jointAccMax(i) - .5*dt*dt;
//        _c_inf(i) = -_q(i) - dt*_qdot(i) + _jointLimitsMin(i) + .5*_qdot(i)*_qdot(i)/_jointAccMax(i);
        
//        _delta_inf(i) = _b_inf(i)*_b_inf(i) - 4*_a(i)*_c_inf(i);
       
//        if (_delta_inf(i) < 0)
//        {
//            _delta_inf(i) = 0;
//        }
        
//        _ub_inf(i) = .5/_a(i)*(-_b_inf(i) + std::sqrt(_delta_inf(i)));
//        _lb_inf(i) = .5/_a(i)*(-_b_inf(i) - std::sqrt(_delta_inf(i)));
        
   
//        _ub(i) = std::min(_ub_sup(i),_ub_inf(i));
//        _lb(i) = std::max(_lb_sup(i),_lb_inf(i));
        
//        if (_ub(i) < _lb(i))
//        {
//            _ub(i) = _lb(i);
//        }
                    
//        __upperBound(i) =  _ub(i);
//        __lowerBound(i) =  _lb(i);
                     
//    }
    
       
    _generic_constraint_internal->setBounds(__upperBound, __lowerBound);
     
    _generic_constraint_internal->update();
     
    _Aineq = _generic_constraint_internal->getAineq();
    _bLowerBound = _generic_constraint_internal->getbLowerBound();
    _bUpperBound = _generic_constraint_internal->getbUpperBound();

}

void JointLimits::setJointAccMax(const Eigen::VectorXd &jointAccMax)
{
    _jointAccMax = jointAccMax;
}

bool JointLimits::setPStepAheadPredictor(const double p)
{
    if(p < 1.)
        return false;
    _p = p;
    return true;
}

