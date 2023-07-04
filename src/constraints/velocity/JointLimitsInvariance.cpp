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
    _p(1.),
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

    double dt = _p*_dt;

    for(unsigned int i = 0; i < _upperBound.size(); ++i)
    {
        double d = 2. * _jointAccMax[i] * dt * dt * (_jointLimitsMax[i] - x[i]);
        if(d < 0.)
            _upperBound[i] = std::min(-sqrt(fabs(d)), dt * dt * _jointAccMax[i] + dt*_qdot_prev[i]);
        else
            _upperBound[i] = std::min(sqrt(d), dt * dt * _jointAccMax[i] + dt*_qdot_prev[i]);


        d = 2. * _jointAccMax[i] * dt * dt * (-_jointLimitsMin[i] + x[i]);
        if(d < 0.)
            _lowerBound[i] = std::max(sqrt(fabs(d)), dt * dt * -_jointAccMax[i] + dt*_qdot_prev[i]);
        else
            _lowerBound[i] = std::max(-sqrt(d), dt * dt * -_jointAccMax[i] + dt*_qdot_prev[i]);

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

bool JointLimitsInvariance::setPStepAheadPredictor(const double p)
{
    if(p > 1.)
    {
        _p = p;
        return true;
    }
    return false;
}
