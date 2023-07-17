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

JointLimitsInvariance::JointLimitsInvariance(const Eigen::VectorXd &q,
                                             const Eigen::VectorXd &jointBoundMax,
                                             const Eigen::VectorXd &jointBoundMin,
                                             const Eigen::VectorXd &jointAccMax,
                                             XBot::ModelInterface& robot,
                                             const double dt):
    Constraint("joint_limits_invariance", q.size()),
    _dt(dt),
    _jointLimitsMin(jointBoundMin),
    _jointLimitsMax(jointBoundMax),
    _jointAccMax(jointAccMax),
    _robot(robot)
{
    _upperBound.setZero(q.size());
    _lowerBound.setZero(q.size());

    update(q);
}

void JointLimitsInvariance::update(const Eigen::VectorXd &x)
{
    _robot.getJointVelocity(_qdot_prev);

    for(unsigned int i = 0; i < _upperBound.size(); ++i)
    {
        if(_qdot_prev[i] <= 0.)
            _upperBound[i] = std::min(_jointLimitsMax[i] - x[i], _dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i]);
        else
        {
            double d = 2. * _jointAccMax[i] * _dt * _dt * (_jointLimitsMax[i] - x[i]);
            if(d < 0.)
                _upperBound[i] = std::min(-sqrt(fabs(d)), _dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i]);
            else
                _upperBound[i] = std::min(sqrt(d), _dt * _dt * _jointAccMax[i] + _dt*_qdot_prev[i]);
        }

        if(_qdot_prev[i] >= 0.)
            _lowerBound[i] = std::max(_jointLimitsMin[i] - x[i], _dt * _dt * -_jointAccMax[i] + _dt*_qdot_prev[i]);
        else
        {
            double d = 2. * -_jointAccMax[i] * _dt * _dt * (_jointLimitsMin[i] - x[i]);
            if(d < 0.)
                _lowerBound[i] = std::max(sqrt(fabs(d)), _dt * _dt * -_jointAccMax[i] + _dt*_qdot_prev[i]);
            else
                _lowerBound[i] = std::max(-sqrt(d), _dt * _dt * -_jointAccMax[i] + _dt*_qdot_prev[i]);
        }


        if(_lowerBound[i] > _upperBound[i])
        {
            double tmp = _lowerBound[i];
            _lowerBound[i] = _upperBound[i];
            _upperBound[i] = tmp;
        }

    }





}

void JointLimitsInvariance::setJointAccMax(const Eigen::VectorXd& jointAccMax)
{
    _jointAccMax = jointAccMax;
}


