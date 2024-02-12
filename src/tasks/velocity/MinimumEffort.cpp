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

#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;


MinimumEffort::MinimumEffort(const XBot::ModelInterface& robot_model, const double step) :
    Task("min_effort", robot_model.getNv()),
    _model(robot_model),
    _gTauGradientWorker(robot_model),
    _step(step)
{
    _W.resize(robot_model.getNv(), robot_model.getNv());
    _W.setIdentity();

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(Eigen::VectorXd(0));
}

MinimumEffort::~MinimumEffort()
{

}

void MinimumEffort::_update(const Eigen::VectorXd &x) {
    _q = _model.getJointPosition();

    /************************* COMPUTING TASK *****************************/


    Eigen::VectorXd gradient(_model.getNv());
    gradient.setZero();
    Eigen::VectorXd deltas(_model.getNv());
    deltas.setZero();


    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        if(this->getActiveJointsMask()[i])
        {
            deltas[i] = _step;
            double fun_a = _gTauGradientWorker.compute(_model.sum(_q, deltas));
            double fun_b = _gTauGradientWorker.compute(_model.sum(_q, -deltas));

            gradient[i] = (fun_a - fun_b)/(2.0*_step);
            deltas[i] = 0.0;
        } else
            gradient[i] = 0.0;
    }

    _b = -1.0 * _lambda * gradient;

    /**********************************************************************/
}

double MinimumEffort::computeEffort()
{
    return _gTauGradientWorker.compute(_model.getJointPosition());
}



