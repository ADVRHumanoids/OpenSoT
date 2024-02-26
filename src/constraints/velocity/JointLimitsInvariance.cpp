/*
 * Copyright (C) 2023 Enrico Mingo Hoffman
 * Author: Enrico Mingo Hoffman
 * email:  enricomingo@gmail.com
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

#include <OpenSoT/constraints/velocity/JointLimitsInvariace.h>

using namespace OpenSoT::constraints::velocity;

JointLimitsInvariance::JointLimitsInvariance(const Eigen::VectorXd &jointBoundMax,
                                             const Eigen::VectorXd &jointBoundMin,
                                             const Eigen::VectorXd &jointAccMax,
                                             XBot::ModelInterface& robot,
                                             const double dt):
    Constraint("joint_limits_invariance", robot.getNv()),
    _dt(dt),
    _jointLimitsMin(jointBoundMin),
    _jointLimitsMax(jointBoundMax),
    _jointAccMax(jointAccMax),
    _robot(robot),
    _p(1.),
    _lb(.0), _ub(0.), _acc_lim(0.), _via_lim(0.), _d(0.), _ac_lb(0), _ac_ub(0)
{

    _upperBound.setZero(robot.getNv());
    _lowerBound.setZero(robot.getNv());
    _active_constraint_lb.setZero(robot.getNv());
    _active_constraint_ub.setZero(robot.getNv());

    _zeros = _robot.getNeutralQ();

    update(Eigen::VectorXd(0));
}

void JointLimitsInvariance::update(const Eigen::VectorXd &x)
{
    _robot.getJointVelocity(_qdot_prev);
    _robot.getJointPosition(_q);

    _pos_lim_sup = _jointLimitsMax - _robot.difference(_q, _zeros);
    _pos_lim_inf = _jointLimitsMin - _robot.difference(_q, _zeros);


    for(unsigned int i = 0; i < _upperBound.size(); ++i)
    {
        _acc_lim = _dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i];
        if(_qdot_prev[i] <= 0.)
        {
            if(_pos_lim_sup[i] < _acc_lim)
            {
                _ub = _pos_lim_sup[i];
                _ac_ub = active_constraint::pos_lim;
            }
            else
            {
                _ub = _acc_lim;
                _ac_ub = active_constraint::acc_lim;
            }
        }
        else
        {
            _d = 2. * _jointAccMax[i] * _dt * _dt * _p * _pos_lim_sup[i];
            _via_lim = (_d < 0.) ? -sqrt(fabs(_d)) : sqrt(_d);
            if(_via_lim < _acc_lim)
            {
                _ub = _via_lim;
                _ac_ub = active_constraint::via_lim;
            }
            else
            {
                _ub = _acc_lim;
                _ac_ub = active_constraint::pos_lim;
            }

        }

        _acc_lim = -_dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i];
        if(_qdot_prev[i] >= 0.)
        {
            if(_pos_lim_inf[i] > _acc_lim)
            {
                _lb = _pos_lim_inf[i];
                _ac_lb = active_constraint::pos_lim;
            }
            else
            {
                _lb = _acc_lim;
                _ac_lb = active_constraint::acc_lim;
            }
        }
        else
        {
            _d = 2. * -_jointAccMax[i] * _dt * _dt * _p * _pos_lim_inf[i];
            _via_lim = (_d < 0.) ? sqrt(fabs(_d)) : -sqrt(_d);
            if(_via_lim > _acc_lim)
            {
                _lb = _via_lim;
                _ac_lb = active_constraint::via_lim;
            }
            else
            {
                _lb = _acc_lim;
                _ac_lb = active_constraint::via_lim;
            }
        }




// Previous implementation
//    for(unsigned int i = 0; i < _upperBound.size(); ++i)
//    {
//        _acc_lim = _dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i];
//        _pos_lim = _jointLimitsMax[i] - x[i];
//        if(_qdot_prev[i] <= 0.)
//        {
//            if(_pos_lim < _acc_lim)
//            {
//                _ub = _pos_lim;
//                _ac_ub = active_constraint::pos_lim;
//            }
//            else
//            {
//                _ub = _acc_lim;
//                _ac_ub = active_constraint::acc_lim;
//            }
//        }
//        else
//        {
//            _d = 2. * _jointAccMax[i] * _dt * _dt * _p * _pos_lim;
//            _via_lim = (_d < 0.) ? -sqrt(fabs(_d)) : sqrt(_d);
//            if(_via_lim < _acc_lim)
//            {
//                _ub = _via_lim;
//                _ac_ub = active_constraint::via_lim;
//            }
//            else
//            {
//                _ub = _acc_lim;
//                _ac_ub = active_constraint::pos_lim;
//            }

//        }

//        _acc_lim = -_dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i];
//        _pos_lim = _jointLimitsMin[i] - x[i];
//        if(_qdot_prev[i] >= 0.)
//        {
//            if(_pos_lim > _acc_lim)
//            {
//                _lb = _pos_lim;
//                _ac_lb = active_constraint::pos_lim;
//            }
//            else
//            {
//                _lb = _acc_lim;
//                _ac_lb = active_constraint::acc_lim;
//            }
//        }
//        else
//        {
//            _d = 2. * -_jointAccMax[i] * _dt * _dt * _p * _pos_lim;
//            _via_lim = (_d < 0.) ? sqrt(fabs(_d)) : -sqrt(_d);
//            if(_via_lim > _acc_lim)
//            {
//                _lb = _via_lim;
//                _ac_lb = active_constraint::via_lim;
//            }
//            else
//            {
//                _lb = _acc_lim;
//                _ac_lb = active_constraint::via_lim;
//            }
//        }


        if(_lb > _ub)
        {
            _lowerBound[i] = _ub;
            _upperBound[i] = _lb;
            _active_constraint_lb[i] = _ac_ub;
            _active_constraint_ub[i] = _ac_lb;
        }
        else
        {
            _lowerBound[i] = _lb;
            _upperBound[i] = _ub;
            _active_constraint_lb[i] = _ac_lb;
            _active_constraint_ub[i] = _ac_ub;
        }

    }

}

bool JointLimitsInvariance::setPStepAheadPredictor(const double p)
{
    if(p > 1.)
        return false;
    else
        _p = p;
    return true;
}

void JointLimitsInvariance::setJointAccMax(const Eigen::VectorXd& jointAccMax)
{
    _jointAccMax = jointAccMax;
}


