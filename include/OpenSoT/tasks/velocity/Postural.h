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

#ifndef __TASKS_VELOCITY_POSTURAL_H__
#define __TASKS_VELOCITY_POSTURAL_H__

#include <OpenSoT/Task.h>

#include <xbot2_interface/xbotinterface2.h>

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Postural class implements a task that tries to bring the robust posture to a reference posture.
             * You can see an example of it in @ref example_postural.cpp
             */
            class Postural : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef std::shared_ptr<Postural> Ptr;
            protected:
                Eigen::VectorXd _q_desired;
                Eigen::VectorXd _dq;
                Eigen::VectorXd _v_desired, _v_desired_ref;
                Eigen::VectorXd _q;
                const XBot::ModelInterface& _robot;

                void update_b();

            public:

                Postural(const XBot::ModelInterface& robot,
                         const Eigen::VectorXd& x,
                         const std::string& task_id = "Postural");

                ~Postural();

                virtual void _update(const Eigen::VectorXd& x);

                /**
                 * @brief setReference sets a new reference for the Postural task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired position, meaning we are trying to achieve a regulation task.
                 * @param x_desired the \f$R^{n_x}\f$ vector describing the desired joint position
                 */
                void setReference(const Eigen::VectorXd& x_desired);

                /**
                 * @brief setReference sets a new reference for the Postural task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(x_desired, xdot_desired) needs to be called before each _update(x)
                 * of the Postural task, since THE _update() RESETS THE FEED-FORWARD VELOCITY TERM for safety reasons.
                 * @param x_desired the \f$R^{n_x}\f$ vector of desired joint positions.
                 * @param xdot_desired is a \f$R^{n_x}\f$ vector describing the desired joint velocities,
                 * and it represents a feed-forward term in the Postural task computation NOTICE how the velocities are in rad/sample,
                 * instead of rad/s. This means that if you have a velocity expressed in SI units, you have to call the function as
                 * setReference(x_desired, xdot_desired*dt)
                 */
                void setReference(const Eigen::VectorXd& x_desired,
                                  const Eigen::VectorXd& xdot_desired);

                /**
                 * @brief getReference returns the Postural task reference
                 * @return the \f$R^{n_x}\f$ Postural task reference
                 */
                const Eigen::VectorXd& getReference() const;

                /**
                 * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
                 * next update()
                 * @return internal velcity reference
                 */
                const Eigen::VectorXd& getCachedVelocityReference() const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the Postural task.
                 * @param x_desired the \f$R^{n_x}\f$ vector describing the desired position of the COM
                 * in the world coordinate frame.
                 * @param xdot_desired is a \f$R^{n_x}\f$ twist describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the task computation
                 */
                void getReference(Eigen::VectorXd& x_desired,
                                  Eigen::VectorXd& xdot_desired) const;

                void setLambda(double lambda);

                /**
                 * @brief getActualPositions return the actual state position of the task
                 * @return vector of joints positions
                 */
                Eigen::VectorXd getActualPositions();

                /**
                 * @brief getError return the error between the desired and actual joint position values
                 * @return vector of errors
                 */
                Eigen::VectorXd getError();

                /**
                 * @brief reset set as actual joint reference the actual pose
                 * @return
                 */
                bool reset();

                virtual void _log(XBot::MatLogger2::Ptr logger);

                static bool isPostural(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                static OpenSoT::tasks::velocity::Postural::Ptr asPostural(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

            };
        }
    }
 }

#endif
