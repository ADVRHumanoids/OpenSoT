/*
 * Copyright (C) 2014 Walkman
 * Author: Arturo Laurenzi, Enrico Mingo Hoffman
 * email:  arturo.laurenzi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/constraints/torque/JointLimits.h>

using namespace OpenSoT::constraints::torque;

JointLimits::JointLimits(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const XBot::ModelInterface& model) :
    Constraint("joint_limits", q.size()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _model(model)
{

    _k.setConstant(q.size(), 15000);
    _d.setConstant(q.size(), 1000);


    assert(q.rows() == _jointLimitsMax.rows());
    assert(q.rows() == _jointLimitsMin.rows());
    /* calling update to generate bounds */
    update(q);
}

void JointLimits::update(const Eigen::VectorXd& x)
{


/************************ COMPUTING BOUNDS ****************************/

    _model.getJointPosition(_q);
    _model.getJointVelocity(_qdot);
    _upperBound = _k.array() * (_jointLimitsMax - _q).array() - _d.array() * _qdot.array();
    _lowerBound = _k.array() * (_jointLimitsMin - _q).array() - _d.array() * _qdot.array();

/**********************************************************************/

}

void JointLimits::getGains(Eigen::VectorXd& k, Eigen::VectorXd& d) const
{
    k = _k;
    d = _d;
}

void JointLimits::setGains(const Eigen::VectorXd& k, const Eigen::VectorXd& d)
{
    if( (k.array() < 0).any() ) return;
    if( (d.array() < 0).any() ) return;

    _k = k;
    _d = d;
}

void JointLimits::setGains(double k, double d)
{
    if( k < 0 || d < 0 ) return;

    _k.setConstant(_k.size(), k);
    _d.setConstant(_d.size(), d);

}

