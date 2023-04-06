/*
 * Copyright (C) Unmanned Systems and Robotics Lab
 * Author: Andrea Testa
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

#include <OpenSoT/constraints/acceleration/JointLimitsViability.h>
#include <memory>

using namespace OpenSoT::constraints::acceleration;

JointLimitsViability::JointLimitsViability(XBot::ModelInterface& robot,
                                            const AffineHelper& qddot,
                                            const Eigen::VectorXd &jointBoundMax,
                                            const Eigen::VectorXd &jointBoundMin,
                                            const Eigen::VectorXd &jointVelMax,
                                            const Eigen::VectorXd &jointAccMax,
                                            const double dt):
     Constraint("joint_limits_viability", qddot.getInputSize()),
     _jointLimitsMax(jointBoundMax),
     _jointLimitsMin(jointBoundMin),
     _jointVelMax(jointVelMax),
     _jointAccMax(jointAccMax),
     _robot(robot),
     _dt(dt)
{
    if(qddot.getOutputSize() != _jointLimitsMax.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMax.size()");
    if(qddot.getOutputSize() != _jointLimitsMin.size())
        throw std::runtime_error("_qddot.getOutputSize() != _jointLimitsMin.size()");
    if(_jointAccMax.size() != _jointLimitsMin.size())
        throw std::runtime_error("_jointAccMax.size() != _jointLimitsMin.size()");
    if(_jointVelMax.size() != _jointLimitsMin.size())
        throw std::runtime_error("_jointVelMax.size() != _jointLimitsMin.size()");
    /* calling update to generate bounds */

    _ddq_LB_pos.setZero(jointBoundMax.size());
    _ddq_UB_pos.setZero(jointBoundMax.size());
    _ddq_LB_via.setZero(jointBoundMax.size());
    _ddq_UB_via.setZero(jointBoundMax.size());
    _ddq_LB_vel.setZero(jointBoundMax.size());
    _ddq_UB_vel.setZero(jointBoundMax.size());
    _a = _dt*_dt;

    __upperBound =  _jointAccMax;
    __lowerBound = -_jointAccMax;

    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                _jointAccMax,
                -_jointAccMax,
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    update(Eigen::VectorXd(1));
}

void JointLimitsViability::_log(XBot::MatLogger2::Ptr logger)
{
    logger->add("_ddq_LB_pos", _ddq_LB_pos);
    logger->add("_ddq_UB_pos", _ddq_UB_pos);
    logger->add("_ddq_LB_vel", _ddq_LB_vel);
    logger->add("_ddq_UB_vel", _ddq_UB_vel);
    logger->add("_ddq_LB_via", _ddq_LB_via);
    logger->add("_ddq_UB_via", _ddq_UB_via);
    logger->add("__upperBound", __upperBound);
    logger->add("__lowerBound", __lowerBound);
}

void JointLimitsViability::update(const Eigen::VectorXd &x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);

    accBoundsFromPosLimits();
    accBoundsFromViability();
    computeJointAccBounds();

    _generic_constraint_internal->setBounds(__upperBound, __lowerBound);

    _generic_constraint_internal->update(x);

    _Aineq = _generic_constraint_internal->getAineq();
    _bLowerBound = _generic_constraint_internal->getbLowerBound();
    _bUpperBound = _generic_constraint_internal->getbUpperBound();
}

void JointLimitsViability::computeJointAccBounds()
{
    _ddq_UB_vel = (_jointVelMax - _qdot)/_dt;
    _ddq_LB_vel = (-_jointVelMax - _qdot)/_dt;

    for(unsigned int it = 0; it < _jointLimitsMax.size(); ++it)
    {
        __upperBound[it] = std::min(std::min(std::min(_ddq_UB_pos[it], _ddq_UB_vel[it]), _ddq_UB_via[it]), _jointAccMax[it]);
        __lowerBound[it] = std::max(std::max(std::max(_ddq_LB_pos[it], _ddq_LB_vel[it]), _ddq_LB_via[it]), -_jointAccMax[it]);

        if(__upperBound[it] < __lowerBound[it])
        {
            double ub = __upperBound[it];
            __upperBound[it] = __lowerBound[it];
            __lowerBound[it] = ub;
        }

        if(__lowerBound[it] < -_jointAccMax[it])
        {
            __lowerBound[it] = -_jointAccMax[it];
        }

        if(__upperBound[it] > _jointAccMax[it])
        {
            __upperBound[it] = _jointAccMax[it];
        }

    }
}

void JointLimitsViability::accBoundsFromPosLimits()
{
    _ddq_M1 = -_qdot/_dt;
    _ddq_M2 = -(_qdot.array() * _qdot.array()) / (2.0*(_jointLimitsMax.array() - _q.array()));
    _ddq_M3 = 2.0*(_jointLimitsMax - _q - _dt*_qdot)/_a;
    _ddq_m2 = (_qdot.array()*_qdot.array())/(2.0*(_q.array() - _jointLimitsMin.array()));
    _ddq_m3 = 2.0*(_jointLimitsMin - _q - _dt*_qdot)/_a;


    for(unsigned int it = 0; it < _jointLimitsMax.size(); ++it)
    {
        if(_qdot[it] >= 0.)
        {
            _ddq_LB_pos[it] = _ddq_m3[it];
            if(_ddq_M3[it] > _ddq_M1[it])
            {
                _ddq_UB_pos[it] = _ddq_M3[it];
            }
            else
            {
                _ddq_UB_pos[it] = std::min(_ddq_M1[it], _ddq_M2[it]);
            }
        }
        else
        {
            _ddq_UB_pos[it] = _ddq_M3[it];
            if(_ddq_m3[it] < _ddq_M1[it])
            {
                _ddq_LB_pos[it] = _ddq_m3[it];
            }
            else
            {
                _ddq_LB_pos[it] = std::max(_ddq_M1[it], _ddq_m2[it]);
            }
        }
    }
}

void JointLimitsViability::accBoundsFromViability()
{
    _b_1 = _dt*(2.0*_qdot + _jointAccMax*_dt);
    _c_1 = _qdot.array()*_qdot.array() - 2.0*_jointAccMax.array()*(_jointLimitsMax.array() - _q.array() - _dt*_qdot.array());
    _ddq_1 = -_qdot/_dt;
    _delta_1 = _b_1.array()*_b_1.array() - 4.0*_a*_c_1.array();


    for(unsigned int it = 0; it < _jointLimitsMax.size(); ++it)
    {
        if(_delta_1[it] >= 0.)
        {
            _ddq_UB_via[it] = std::max(_ddq_1[it], ( -_b_1[it] + std::sqrt(_delta_1[it]) )/(2.0*_a) );
        }
        else
        {
            _ddq_UB_via[it] = _ddq_1[it];
        }
    }


    _b_2 = _dt*(2.0*_qdot - _jointAccMax*_dt);
    _c_2 = _qdot.array()*_qdot.array() - 2.0*_jointAccMax.array()*(_q.array() + _dt*_qdot.array() - _jointLimitsMin.array());
    _delta_2 = _b_2.array()*_b_2.array() - 4.0*_a*_c_2.array();


    for(unsigned int it = 0; it < _jointLimitsMax.size(); ++it)
    {
        if(_delta_2[it] >= 0.)
        {
            _ddq_LB_via[it] = std::min(_ddq_1[it], ( -_b_2[it] - std::sqrt(_delta_2[it]) )/(2.0*_a) );
        }
        else
        {
            _ddq_LB_via[it] = _ddq_1[it];
        }
    }
}

void JointLimitsViability::setJointAccMax(const Eigen::VectorXd &jointAccMax)
{
    _jointAccMax = jointAccMax;
}

void JointLimitsViability::setJointVelMax(const Eigen::VectorXd& jointVelMax)
{
    _jointVelMax = jointVelMax;
}
