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

#include <wb_sot/tasks/velocity/MinimumEffort.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace wb_sot::tasks::velocity;
using namespace yarp::math;

MinimumEffort::MinimumEffort(   const yarp::sig::Vector& x) :
    Task("posture", x, x.size())
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    /* first update. Setting desired pose equal to the actual pose */
    this->update(_x0);
}

MinimumEffort::~MinimumEffort()
{
   //_referenceInputPort.close();
}

void MinimumEffort::update(const yarp::sig::Vector &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    yarp::sig::Matrix W(_robot.coman_iDyn3.getJointTorqueMax().size(), _robot.coman_iDyn3.getJointTorqueMax().size());
    W.eye();
    for(unsigned int i = 0; i < _robot.coman_iDyn3.getJointTorqueMax().size(); ++i)
        W(i,i) = 1.0 / (_robot.coman_iDyn3.getJointTorqueMax()[i]*_robot.coman_iDyn3.getJointTorqueMax()[i]);
    _b = -1.0 * getGravityCompensationGradient(W);

    /**********************************************************************/
}


yarp::sig::Vector MinimumEffort::getGravityCompensationTorque(const yarp::sig::Vector q)
{
    static yarp::sig::Vector zeroes(_x.size(),0.0);
    static yarp::sig::Vector tau(_x.size(),0.0);

    _robot.updateiDyn3Model(_x, true);

    tau = _robot.coman_iDyn3.getTorques();

    return tau;
}

/**
 * Two-point formula: it computes the slope of a nearby secant line through the
 * points (x-h,f(x-h)) and (x+h,f(x+h))
 **/
yarp::sig::Vector MinimumEffort::getGravityCompensationGradient(const yarp::sig::Matrix& W)
{
    //double start = yarp::os::Time::now();
    /// cost function is tau_g^t*tau_g
    static yarp::sig::Vector gradient(_robot.coman_iDyn3.getNrOfDOFs(),0.0);
    static yarp::sig::Vector deltas(_robot.coman_iDyn3.getNrOfDOFs(),0.0);
    for(unsigned int i = 0; i < gradient.size(); ++i)
    {
        // forward method gradient computation, milligrad
        const double h = 1E-3;
        deltas[i] = h;
        yarp::sig::Vector tau_gravity_q_a = getGravityCompensationTorque(_x+deltas);
        yarp::sig::Vector tau_gravity_q_b = getGravityCompensationTorque(_x-deltas);

        double C_g_q_a = yarp::math::dot(tau_gravity_q_a, W*tau_gravity_q_a);
        double C_g_q_b = yarp::math::dot(tau_gravity_q_b, W*tau_gravity_q_b);
        gradient[i] = (C_g_q_a - C_g_q_b)/(2*h);
        deltas[i] = 0;
    }

    //double elapsed = yarp::os::Time::now() - start;
    //ROS_WARN(" took %f ms", elapsed);
    return gradient;
}


