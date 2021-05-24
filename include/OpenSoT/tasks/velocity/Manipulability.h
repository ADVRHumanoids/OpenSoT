/*
 * Copyright (C) 2014 Walkman
 * Authors: Enrico Mingo
 * email: enrico.mingo@iit.it
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

#ifndef __TASKS_VELOCITY_MANIPULABILITY_H__
#define __TASKS_VELOCITY_MANIPULABILITY_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>



namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Manipulability class implements a task that tries to maximize the Manipulability
             * index computed as (Robotics: Modelling, Planning and Control, pag. 126):
             *
             *              w = sqrt(det(J*W*J'))
             *
             * The gradient of w is then computed and projected using the gardient projection method.
             * W is a CONSTANT weight matrix.
             */
            class Manipulability : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef std::shared_ptr<Manipulability> Ptr;

                Manipulability(const Eigen::VectorXd& x, const XBot::ModelInterface& robot_model, const Cartesian::Ptr CartesianTask);
                Manipulability(const Eigen::VectorXd& x, const XBot::ModelInterface& robot_model, const CoM::Ptr CartesianTask);

                ~Manipulability();

                void _update(const Eigen::VectorXd& x);

                /**
                 * @brief ComputeManipulabilityIndex
                 * @return the manipulability index at the actual configuration q (ast from latest update(q))
                 */
                double ComputeManipulabilityIndex();

                /**
                 * @brief setW set a CONSTANT Weight matrix for the manipulability index
                 * @param W weight matrix
                 */
                void setW(const Eigen::MatrixXd& W){
                    _manipulabilityIndexGradientWorker.setW(W);
                }

                /**
                 * @brief getW get a Weight matrix for the manipulability index
                 */
                Eigen::MatrixXd getW(){
                    return _manipulabilityIndexGradientWorker.getW();
                }

                void setLambda(double lambda)
                {
                    if(lambda >= 0.0){
                        _lambda = lambda;
                        this->_update(_x);
                    }
                }

            protected:

                Eigen::VectorXd _x;

                /**
                 * @brief The ComputeManipulabilityIndexGradient class implements a worker class to computes
                 * the effort for a certain configuration.
                 */
                class ComputeManipulabilityIndexGradient: public CostFunction{
                public:
                    XBot::ModelInterface::Ptr _robot;
                    const XBot::ModelInterface& _model;
                    Eigen::MatrixXd _W;
                    Eigen::VectorXd _zeros;
                    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr _CartesianTask;

                    ComputeManipulabilityIndexGradient(const Eigen::VectorXd& q, const XBot::ModelInterface& robot_model,
                                                       const Cartesian::Ptr CartesianTask) :
                        _robot(XBot::ModelInterface::getModel(robot_model.getConfigOptions())),
                        _model(robot_model),
                        _W(q.rows(),q.rows()),
                        _zeros(q.rows())
                    {
                        _W.setIdentity(_W.rows(), _W.cols());

                        _zeros.setZero(_zeros.rows());

                        _robot->syncFrom(_model);

                        _CartesianTask = Cartesian::Ptr(new Cartesian(CartesianTask->getTaskID(), q,
                                                *(_robot.get()), CartesianTask->getDistalLink(), CartesianTask->getBaseLink()));
                    }

                    ComputeManipulabilityIndexGradient(const Eigen::VectorXd& q, const XBot::ModelInterface& robot_model,
                                                       const CoM::Ptr CartesianTask) :
                        _robot(XBot::ModelInterface::getModel(robot_model.getPathToConfig())),
                        _model(robot_model),
                        _W(q.rows(),q.rows()),
                        _zeros(q.rows())
                    {
                        _W.setIdentity(_W.rows(), _W.cols());

                        _zeros.setZero(_zeros.rows());

                        _robot->syncFrom(_model);

                        _CartesianTask = CoM::Ptr(new OpenSoT::tasks::velocity::CoM(q, *(_robot.get())));
                    }

                    double compute(const Eigen::VectorXd &q)
                    {
                        _robot->setJointPosition(q);
                        _robot->update(true);


                        _CartesianTask->update(q);

                        return computeManipulabilityIndex();
                    }

                    void setW(const Eigen::MatrixXd& W) { _W = W; }

                    Eigen::MatrixXd& getW() {return _W;}

                    double computeManipulabilityIndex()
                    {
                        Eigen::MatrixXd J = _CartesianTask->getA();
                        //fabs is to avoid nan when we have -1e-18!
                        return sqrt(fabs((J*_W*J.transpose()).determinant()));
                    }
                };

                ComputeManipulabilityIndexGradient _manipulabilityIndexGradientWorker;
            };
        }
    }
}

#endif
