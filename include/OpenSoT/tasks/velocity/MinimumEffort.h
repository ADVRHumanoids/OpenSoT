
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
 #include <xbot2_interface/xbotinterface2.h>
 #include <OpenSoT/utils/cartesian_utils.h>


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
            class MinimumEffort : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef std::shared_ptr<MinimumEffort> Ptr;
            protected:
                const XBot::ModelInterface& _model;
                Eigen::VectorXd _q;
                /**
                 * @brief The ComputeGTauGradient class implements a worker class to computes the effort for a certain configuration.
                 * It will take into account the robot as standing on a flat floor, and while computing the gradient,
                 * we will assume a foot is on the ground. Notice that the way we compute the gradient does not satisfy the constraints
                 * of both flat foot on the ground. So in general this simple implementation of the gradient worker needs to be used
                 * in a minimum effort task together with a constraint (or a higher priority task) for the swing foot.
                 */
                class ComputeGTauGradient : public CostFunction {
                    public:
                    XBot::ModelInterface::Ptr _robot;
                    const XBot::ModelInterface& _model;
                    Eigen::MatrixXd _W;
                    Eigen::VectorXd _tau;
                    Eigen::VectorXd _tau_lim;

                    ComputeGTauGradient(const XBot::ModelInterface& robot_model) :
                        _robot(robot_model.clone()),
                        _model(robot_model),
                        _W(robot_model.getNv(), robot_model.getNv())
                    {
                        _W.setIdentity();

                        _robot->syncFrom(_model);

                        _model.getEffortLimits(_tau_lim);

                        for(int i = 0; i < robot_model.getNv(); ++i){
                            _W(i,i) = 1.0 / std::pow(_tau_lim(i), 2.0);
                        }

                    }

                    double compute(const Eigen::VectorXd &q)
                    {
                        _robot->setJointPosition(q);
                        _robot->update();

                        _robot->computeGravityCompensation(_tau);
                        return _tau.transpose()* _W * _tau;
                    }

                    void setW(const Eigen::MatrixXd& W) { _W = W; }

                    const Eigen::MatrixXd& getW() const {return _W;}
                };

                ComputeGTauGradient _gTauGradientWorker;
                double _step;
                Eigen::VectorXd _gradient;
                Eigen::VectorXd _deltas;

            public:

                MinimumEffort(const XBot::ModelInterface& robot_model, const double step = 1E-3);

                ~MinimumEffort();

                /**
                 * @brief _update updates the minimum effort gradient.
                 * @detail It also updates the state on the internal robot model so that successive calls to the
                 * computeEffort() function will take into account the updated posture of the robot.
                 * @param x the actual posture of the robot
                 */
                void _update(const Eigen::VectorXd& x);

                /**
                 * @brief computeEffort
                 * @return the effort at the actual configuration q (ast from latest update(q))
                 */
                double computeEffort();

                /**
                 * @brief setW set a CONSTANT Weight matrix for the manipulability index
                 * @param W weight matrix
                 */
                void setW(const Eigen::MatrixXd& W){
                    _gTauGradientWorker.setW(W);
                }

                /**
                 * @brief getW get a Weight matrix for the manipulability index
                 */
                const Eigen::MatrixXd& getW(){
                    return _gTauGradientWorker.getW();
                }

                void setLambda(double lambda)
                {
                    if(lambda >= 0.0){
                        _lambda = lambda;
                        this->_update(Eigen::VectorXd(0));
                    }
                }
            };
        }
    }
 }

#endif
