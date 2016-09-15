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

#ifndef __TASKS_VELOCITY_COM_H__
#define __TASKS_VELOCITY_COM_H__

#include <OpenSoT/Task.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>



/**
  * @example example_com.cpp
  * The CoM class implements a task that tries to impose a position
  * of the CoM w.r.t. the support foot.
  */
 namespace OpenSoT {
    namespace tasks {
        namespace velocity {

        /**
          * Note that this is the frame where you have to specify the velocity for the DISTAL_LINK_COM.
          * The floating_base_link instead is placed in the idynutils model!
          */
        #define BASE_LINK_COM "world"
        #define DISTAL_LINK_COM "CoM"
            /**
             * @brief The CoM class implements a task that tries to impose a position
             * of the CoM w.r.t. the support foot. Notice how you need to use it with a model with
             * the floating base link set as the support foot.
             * You can see an example in @ref example_com.cpp
             */
            class CoM : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<CoM> Ptr;
            private:
                iDynUtils& _robot;

                yarp::sig::Vector _actualPosition;
                yarp::sig::Vector _desiredPosition;
                yarp::sig::Vector _desiredVelocity;

                void update_b();

            public:

                yarp::sig::Vector positionError;

                /**
                 * @brief CoM
                 * @param x the initial configuration of the robot
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoM(const yarp::sig::Vector& x,
                    iDynUtils& robot);

                ~CoM();

                void _update(const yarp::sig::Vector& x);

                /**
                 * @brief setReference sets a new reference for the CoM task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired position, meaning we are trying to achieve a regulation task.
                 * @param desiredPose the \f$R^{3}\f$ vector describing the desired position for the CoM
                 * in the world coordinate frame
                 */
                void setReference(const yarp::sig::Vector& desiredPosition);

                /**
                 * @brief setReference sets a new reference for the CoM task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(desiredPosition, desiredVelocity) needs to be called before each _update(x)
                 * of the CoM task, since the _update() resets the feed-forward velocity term for safety reasons.
                 * @param desiredPosition the \f$R^{3}\f$ vector describing the desired position of the CoM wrt world.
                 * @param desireVelocity is a \f$R^{3}\f$ linear velocity vector describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the CoM task computation. NOTICE how the velocities are in m/sample,
                 * instead of m/s. This means that if you have a linear velocity expressed in SI units, you have to call the function as
                 * setReference(desiredPosition, desiredVelocity*dt)
                 */
                void setReference(const yarp::sig::Vector& desiredPosition,
                                  const yarp::sig::Vector& desiredVelocity);


                /**
                 * @brief getReference returns the CoM task reference
                 * @return the CoM task reference \f$R^3\f$ vector describing the actual
                 * CoM position in the world coordinate frame
                 */
                yarp::sig::Vector getReference() const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the CoM task.
                 * @param desiredPosition the \f$R^{3}\f$ vector describing the desired position of the CoM
                 * in the world coordinate frame.
                 * @param desireVelocity is a \f$R^{3}\f$ twist describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the task computation
                 */
                void getReference(yarp::sig::Vector& desiredPosition,
                                  yarp::sig::Vector& desiredVelocity) const;


                /**
                 * @brief getActualPosition returns the CoM actual position. You need to call _update(x) for the position to change
                 * @return the \f$R^{3}\f$ vector describing the actual CoM position in the world coordinate frame
                 */
                yarp::sig::Vector getActualPosition() const;

                /**
                 * @brief getBaseLink an utility function that always returns "world"
                 * @return "world"
                 */
                std::string getBaseLink();

                /**
                 * @brief getDistalLink an utility function that always
                 * @return
                 */
                std::string getDistalLink();

                void setLambda(double lambda);

                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                yarp::sig::Vector getError();

                static bool isCoM(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
                {
                    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
                }

                static OpenSoT::tasks::velocity::CoM::Ptr asCoM(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
                {
                    return boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(task);
                }
            };
        }
    }
 }

#endif
