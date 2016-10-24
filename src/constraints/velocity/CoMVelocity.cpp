/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

CoMVelocity::CoMVelocity(const Eigen::VectorXd velocityLimits,
                         const double dT,
                         const Eigen::VectorXd& x,
                         iDynUtils &robot) :
Constraint("CoMVelocity", x.size()), _dT(dT), _velocityLimits(velocityLimits),
_robot(robot) {

    if(_velocityLimits.size() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";

    _Aineq.resize(3, x.size());
    _bLowerBound.resize(x.size());
    _bUpperBound.resize(x.size());

    this->generatebBounds();

    this->update(x);
}

void CoMVelocity::update(const Eigen::VectorXd &x) {

    Eigen::MatrixXd JCoM;
    if(!_robot.getCOMJacobian(JCoM))
        throw "Error computing CoM Jacobian";
    JCoM = JCoM.block(0,6,3,_x_size);

    /************************ COMPUTING BOUNDS ****************************/

    _Aineq = JCoM;

    /**********************************************************************/
}

Eigen::VectorXd OpenSoT::constraints::velocity::CoMVelocity::getVelocityLimits()
{
    return _velocityLimits;
}

void OpenSoT::constraints::velocity::CoMVelocity::setVelocityLimits(const Eigen::VectorXd velocityLimits)
{
    if(_velocityLimits.size() < 3 )
        throw "Error: velocityLimits for CoM should be a vector of 3 elements";
    _velocityLimits = velocityLimits;
}

void CoMVelocity::generatebBounds() {

    /******************** COMPUTING CONSTANT BOUNDS ***********************/

    _bLowerBound = -1.0*_velocityLimits*_dT;
    _bUpperBound = +1.0*_velocityLimits*_dT;

    /**********************************************************************/
}


