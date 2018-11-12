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

 #include <kdl/frames.hpp>
#include <Eigen/Dense>

/**
 * @example example_postural.cpp
 * The Postural class implements a task that tries to bring the robust posture to a reference posture.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Postural class implements a task that tries to bring the robust posture to a reference posture.
             * You can see an example of it in @ref example_postural.cpp
             */
            class Postural : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<Postural> Ptr;
            protected:
                Eigen::VectorXd _x_desired;
                Eigen::VectorXd _xdot_desired;
                Eigen::VectorXd _x;

                void update_b();

            public:

                Postural(const Eigen::VectorXd& x);

                ~Postural();

                void _update(const Eigen::VectorXd& x);

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
                 * of the Postural task, since the _update() resets the feed-forward velocity term for safety reasons.
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
                Eigen::VectorXd getReference() const;

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

            };
        }
    }
 }

#endif
