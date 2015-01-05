
/*
 * Copyright (C) 2014 Walkman
 * Authors:Alessio Rocchi, Enrico Mingo
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

#ifndef __TASKS_VELOCITY_MINIMUMEFFORT_H__
#define __TASKS_VELOCITY_MINIMUMEFFORT_H__

 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/cartesian_utils.h>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>

 using namespace yarp::math;

/**
  * @example example_minimum_effort.cpp
  * The MinimumEffort class implements a task that tries to bring the robot in a minimum-effort posture.
  */
 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The MinimumEffort class implements a task that tries to bring the robot in a minimum-effort posture.
             * Notice that the weight of the robot is not taken into account when computing effort on the legs
             * (that is, the forces on the floating base are not projected on the contact points jacobians).
             * Also, the minimum effort task is using a simple gradient worker, ComputeGTauGradient, which does not satisfy contact points constraints
             * while performing the configuration vector needed to numerically compute the gradient. In particular, the gravity vector
             * is computed considering a support foot always in contact with the ground.
             * This means in general the minimum effort task should be used together with a cartesian task on the swing foot, imeplemented
             * through the OpenSoT::tasks::velocity::Cartesian class.
             * You can take a look at an implementation example in @ref example_minimum_effort.cpp
             */
            class MinimumEffort : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<MinimumEffort> Ptr;
            protected:
                yarp::sig::Vector _x;

                /**
                 * @brief The ComputeGTauGradient class implements a worker class to computes the effort for a certain configuration.
                 * It will take into account the robot as standing on a flat floor, and while computing the gradient,
                 * we will assume a foot is on the ground. Notice that the way we compute the gradient does not satisfy the constraints
                 * of both flat foot on the ground. So in general this simple implementation of the gradient worker needs to be used
                 * in a minimum effort task together with a constraint (or a higher priority task) for the swing foot.
                 */
                class ComputeGTauGradient : public cartesian_utils::CostFunction {
                    public:
                    iDynUtils _robot;
                    yarp::sig::Matrix _W;
                    yarp::sig::Vector _zeros;

                    ComputeGTauGradient(const yarp::sig::Vector& q, const iDynUtils robot_model) :
                        _robot(robot_model),
                        _W(q.size(),q.size()),
                        _zeros(q.size(), 0.0)
                    {
                        _W.eye();

                        for(unsigned int i = 0; i < q.size(); ++i)
                            _W(i,i) = 1.0 / (_robot.iDyn3_model.getJointTorqueMax()[i]
                                                            *
                                             _robot.iDyn3_model.getJointTorqueMax()[i]);
                    }

                    double compute(const yarp::sig::Vector &q)
                    {
                        _robot.updateiDyn3Model(q, _zeros, _zeros, true);
                        yarp::sig::Vector tau = _robot.iDyn3_model.getTorques();
                        return yarp::math::dot(tau, _W * tau);
                    }

                    void setW(const yarp::sig::Matrix& W) { _W = W; }

                    yarp::sig::Matrix& getW() {return _W;}
                };

                ComputeGTauGradient _gTauGradientWorker;

            public:

                MinimumEffort(const yarp::sig::Vector& x, const iDynUtils& robot_model);

                ~MinimumEffort();

                /**
                 * @brief _update updates the minimum effort gradient.
                 * @detail It also updates the state on the internal robot model so that successive calls to the
                 * computeEffort() function will take into account the updated posture of the robot.
                 * @param x the actual posture of the robot
                 */
                void _update(const yarp::sig::Vector& x);

                /**
                 * @brief computeEffort
                 * @return the effort at the actual configuration q (ast from latest update(q))
                 */
                double computeEffort();
            };
        }
    }
 }

#endif
