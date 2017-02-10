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

#include <OpenSoT/tasks/velocity/Postural.h>
#include <exception>
#include <cmath>


using namespace OpenSoT::tasks::velocity;

Postural::Postural(   const Eigen::VectorXd& x) :
    Task("Postural", x.size()), _x(x),
    _x_desired(x.size()), _xdot_desired(x.size())
{
    _x_desired.setZero(_x_size);
    _xdot_desired.setZero(_x_size);

    _W.setIdentity(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->_update(x);
}

Postural::~Postural()
{
}

void Postural::_update(const Eigen::VectorXd &x) {
    _x = x;

    /************************* COMPUTING TASK *****************************/

    this->update_b();

    _xdot_desired.setZero(_x_size);

    /**********************************************************************/
}

void Postural::setReference(const Eigen::VectorXd& x_desired) {
    if(x_desired.size() == _x_size)
    {
        _x_desired = x_desired;
        _xdot_desired.setZero(_x_size);
        this->update_b();
    }
}

void OpenSoT::tasks::velocity::Postural::setReference(const Eigen::VectorXd &x_desired,
                                                      const Eigen::VectorXd &xdot_desired)
{
    if(x_desired.size() == _x_size && xdot_desired.size() == _x_size)
    {
        _x_desired = x_desired;
        _xdot_desired = xdot_desired;
        this->update_b();
    }
}

Eigen::VectorXd OpenSoT::tasks::velocity::Postural::getReference() const
{
    return _x_desired;
}

void OpenSoT::tasks::velocity::Postural::getReference(Eigen::VectorXd &x_desired,
                                                      Eigen::VectorXd &xdot_desired) const
{
    x_desired = _x_desired;
    xdot_desired = _xdot_desired;
}

void Postural::update_b() {
    _b = _xdot_desired + _lambda*(_x_desired - _x);
}

void OpenSoT::tasks::velocity::Postural::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

Eigen::VectorXd Postural::getError()
{
    return _x_desired - _x;
}

Eigen::VectorXd OpenSoT::tasks::velocity::Postural::getActualPositions()
{
    return _x;
}



