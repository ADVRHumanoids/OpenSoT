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
 #include <idynutils/idynutils.h>
 #include <idynutils/convex_hull.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

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
            class Postural : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<Postural> Ptr;
            protected:
                yarp::sig::Vector _x_desired;
                yarp::sig::Vector _xdot_desired;
                yarp::sig::Vector _x;

                void update_b();

            public:

                Postural(const yarp::sig::Vector& x);

                ~Postural();

                void _update(const yarp::sig::Vector& x);

                /**
                 * @brief setReference sets a new reference for the Postural task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired position, meaning we are trying to achieve a regulation task.
                 * @param x_desired the \f$R^{n_x}\f$ vector describing the desired joint position
                 */
                void setReference(const yarp::sig::Vector& x_desired);

                /**
                 * @brief setReference sets a new reference for the Postural task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(x_desired, xdot_desired) needs to be called before each _update(x)
                 * of the Postural task, since the _update() resets the feed-forward velocity term for safety reasons.
                 * @param x_desired the \f$R^{n_x}\f$ vector of desired joint positions.
                 * @param xdot_desired is a \f$R^{n_x}\f$ vector describing the desired joint velocities,
                 * and it represents a feed-forward term in the Postural task computation
                 */
                void setReference(const yarp::sig::Vector& x_desired,
                                  const yarp::sig::Vector& xdot_desired);

                /**
                 * @brief getReference returns the Postural task reference
                 * @return the \f$R^{n_x}\f$ Postural task reference
                 */
                yarp::sig::Vector getReference() const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the Postural task.
                 * @param x_desired the \f$R^{n_x}\f$ vector describing the desired position of the COM
                 * in the world coordinate frame.
                 * @param xdot_desired is a \f$R^{n_x}\f$ twist describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the task computation
                 */
                void getReference(yarp::sig::Vector& x_desired,
                                  yarp::sig::Vector& xdot_desired) const;

                void setLambda(double lambda);
            };
        }
    }
 }

#endif
