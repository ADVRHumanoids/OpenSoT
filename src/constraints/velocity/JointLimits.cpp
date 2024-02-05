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

#include <OpenSoT/constraints/velocity/JointLimits.h>

using namespace OpenSoT::constraints::velocity;

JointLimits::JointLimits(   const XBot::ModelInterface& robot,
                            const Eigen::VectorXd& jointBoundMax,
                            const Eigen::VectorXd& jointBoundMin,
                            const double boundScaling) :
    Constraint("joint_limits", robot.getNv()),
    _robot(robot),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _boundScaling(boundScaling) {


    /* calling update to generate bounds */
    JointLimits::update(Eigen::VectorXd(0));
}

void JointLimits::update(const Eigen::VectorXd& x)
{
    _robot.difference(_robot.getJointPosition(),
                      _robot.getNeutralQ(),
                      _dq);

/************************ COMPUTING BOUNDS ****************************/

    if(_robot.getNv() == _jointLimitsMax.size())
    {
        _upperBound = ( _jointLimitsMax - _dq)*_boundScaling;
        _lowerBound = ( _jointLimitsMin - _dq)*_boundScaling;

        // avoid infeasibility
        _upperBound = _upperBound.cwiseMax(0.0);
        _lowerBound = _lowerBound.cwiseMin(0.0);
    }
    else
        XBot::Logger::warning("Wrong input x size, joint limits will not be upated!\n");

/**********************************************************************/

}

void JointLimits::setBoundScaling(const double boundScaling)
{
    _boundScaling = boundScaling;
}




