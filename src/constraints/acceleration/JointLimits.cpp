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

#include <OpenSoT/constraints/acceleration/JointLimits.h>

using namespace OpenSoT::constraints::velocity::affine;

JointLimits::JointLimits(   const Eigen::VectorXd& q,
                            XBot::ModelInterface& robot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointAccMax):
     Constraint("joint_limits", q.size()),
     _qddot(qddot),
     _jointLimitsMax(jointBoundMax),
     _jointLimitsMin(jointBoundMin)
{

    if(q.size() != _jointLimitsMax.size())
        throw std::runtime_error("q.size() != _jointLimitsMax.size()");
    if(q.size() != _jointLimitsMin.size())
        throw std::runtime_error("q.size() != _jointLimitsMin.size()");
    /* calling update to generate bounds */       

    update(q,qdot,jointAccMax);
}


void JointLimits::update(const Eigen::VectorXd& x, const Eigen::VectorXd& xdot, const Eigen::VectorXd& xddotMax)
{
    
         
        _Aineq = _qddot.getM();
        _bUpperBound =   xddotMax;
        _bLowerBound = - xddotMax;
        
        for(int i = 0; i < x.size(); i++)
        {            
             if (xdot(i) == 0)
             {
                 _invFunUpperBound(i) =  x(i) - _jointLimitsMax(i);
                 _invFunLowerBound(i) = -x(i) + _jointLimitsMin(i);
             }
             else if (xdot(i) < 0)
             {
                 _invFunUpperBound(i) =  x(i) - _jointLimitsMax(i);
                 _invFunLowerBound(i) = -x(i) + _jointLimitsMin(i) - 1/(2*xddotMax(i))*pow(xdot(i),2);
             }
             else
             {
                 _invFunUpperBound(i) =  x(i) - _jointLimitsMax(i) + 1/(2*xddotMax(i))*pow(xdot(i),2);
                 _invFunLowerBound(i) = -x(i) + _jointLimitsMin(i);
             }
             
             if (_invFunUpperBound(i) >= 0)
             {
                _bUpperBound(i) = - xddotMax(i);
                _bLowerBound(i) = - xddotMax(i); 
             }
             
             if (_invFunLowerBound(i) >= 0)
             {
                _bUpperBound(i) =  xddotMax(i);
                _bLowerBound(i) =  xddotMax(i); 
             }
                         
        }
    
}


