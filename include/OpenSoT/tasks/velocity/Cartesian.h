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

#ifndef __TASKS_VELOCITY_CARTESIAN_H__
#define __TASKS_VELOCITY_CARTESIAN_H__

 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 #define WORLD_FRAME_NAME "world"

/**
 * @example example_cartesian.cpp
 * The Cartesian class implements a task that tries to impose a pose (position and orientation)
 * of a distal link w.r.t. a base link.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Cartesian class implements a task that tries to impose a pose (position and orientation)
             * of a distal link w.r.t. a base link. The reference for the cartesian task is set in base link
             * coordinate frame, or in world if the base link name is set to "world".
             * The Cartesian Task is implemented so that
             * \f$A={}^\text{base}J_\text{distal}\f$
             * and
             * \f$b=K_p*e+\frac{1}{\lambda}\xi_d\f$
             *
             * You can see an example in @ref example_cartesian.cpp
             */
            class Cartesian : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<Cartesian> Ptr;
            protected:
                iDynUtils& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                yarp::sig::Matrix _actualPose;
                yarp::sig::Matrix _desiredPose;
                yarp::sig::Vector _desiredTwist;

                bool _base_link_is_world;

                void update_b();

                double _orientationErrorGain;

            public:

                yarp::sig::Vector positionError;
                yarp::sig::Vector orientationError;

                /*********** TASK PARAMETERS ************/



                /****************************************/

                /**
                 * @brief Cartesian creates a new Cartesian task
                 * @param task_id an identifier for the task.
                 * @param x the robot configuration. The Cartesian task will be created so that the task error is zero in position x.
                 * @param robot the robot model. Cartesian expects the robot model to be updated externally.
                 * @param distal_link the name of the distal link as expressed in the robot urdf
                 * @param base_link the name of the base link as expressed in the robot urdf. Can be set to "world"
                 */
                Cartesian(std::string task_id,
                          const yarp::sig::Vector& x,
                          iDynUtils &robot,
                          std::string distal_link,
                          std::string base_link);

                ~Cartesian();

                void _update(const yarp::sig::Vector& x);

                /**
                 * @brief setReference sets a new reference for the Cartesian task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired pose, meaning we are trying to achieve a regulation task.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 */
                void setReference(const yarp::sig::Matrix& desiredPose);

                /**
                 * @brief setReference sets a new reference for the Cartesian task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(desiredPose, desiredTwist) needs to be called before each _update(x) of the Cartesian task,
                 * since the _update() resets the feed-forward velocity term for safety reasons.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 * @param desireVelocity is a \f$R^{6}\f$ twist describing the desired trajectory velocity, and it represents
                 * a feed-forward term in the cartesian task computation. NOTICE how the velocities are in units/sample,
                 * instead of units/s. This means that if you have a twist expressed in SI units, you have to call the function as
                 * setReference(desiredPose, desiredTwist*dt)
                 */
                void setReference(const yarp::sig::Matrix& desiredPose,
                                  const yarp::sig::Vector& desiredTwist);

                /**
                 * @brief getReference returns the Cartesian task reference
                 * @return the Cartesian task reference \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 */
                const yarp::sig::Matrix getReference() const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the Cartesian task.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 * @param desireVelocity is a \f$R^{6}\f$ twist describing the desired trajectory velocity, and it represents
                 * a feed-forward term in the cartesian task computation
                 */
                void getReference(yarp::sig::Matrix& desiredPose,
                                  yarp::sig::Vector& desiredTwist) const;


                /**
                 * @brief getActualPose returns the distal_link actual pose. You need to call _update(x) for the actual pose to change
                 * @return the \f$R^{4x4}\f$ homogeneous transform matrix describing the actual pose
                 * for the distal_link in the base_link frame of reference.
                 */
                const yarp::sig::Matrix getActualPose() const;

                void setOrientationErrorGain(const double& orientationErrorGain);
                const double getOrientationErrorGain() const;

                const std::string getDistalLink() const;
                const std::string getBaseLink() const;

                void setLambda(double lambda);
            };
        }
    }
 }

#endif
