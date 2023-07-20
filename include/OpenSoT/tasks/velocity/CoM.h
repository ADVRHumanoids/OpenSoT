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
#include <XBotInterface/ModelInterface.h>
#include <kdl/frames.hpp>
#include <Eigen/Dense>


 namespace OpenSoT {
    namespace tasks {
        namespace velocity {

        /**
          * Here we hardcode the base_link and distal_link frames
          */
        #define BASE_LINK_COM "world"
        #define DISTAL_LINK_COM "CoM"
            /**
             * @brief The CoM class implements a task that tries to impose a position
             * of the CoM w.r.t. the world frame.
             */
            class CoM : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef std::shared_ptr<CoM> Ptr;
            private:
                XBot::ModelInterface& _robot;

                Eigen::Vector3d _actualPosition;
                Eigen::Vector3d _desiredPosition;
                Eigen::Vector3d _desiredVelocity, _desiredVelocityRef;

                Eigen::Vector3d _positionError;

                void update_b();

                std::string _base_link;
                std::string _distal_link;

            public:



                /**
                 * @brief CoM
                 * @param x the initial configuration of the robot
                 * @param robot the robot model
                 */
                CoM(const Eigen::VectorXd& x,
                    XBot::ModelInterface& robot,
                    const std::string& id = "CoM"
                   );

                ~CoM();

                virtual void _update(const Eigen::VectorXd& x);

                /**
                 * @brief setReference sets a new reference for the CoM task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired position, meaning we are trying to achieve a regulation task.
                 * @param desiredPose the \f$R^{3}\f$ vector describing the desired position for the CoM
                 * in the world coordinate frame
                 */
                virtual void setReference(const Eigen::Vector3d& desiredPosition);
                virtual void setReference(const KDL::Vector& desiredPosition);

                /**
                 * @brief setReference sets a new reference for the CoM task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(desiredPosition, desiredVelocity) needs to be called before each _update(x)
                 * of the CoM task, since THE _update() RESETS THE FEED-FORWARD VELOCITY TERM for safety reasons.
                 * @param desiredPosition the \f$R^{3}\f$ vector describing the desired position of the CoM wrt world.
                 * @param desireVelocity is a \f$R^{3}\f$ linear velocity vector describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the CoM task computation. NOTICE how the velocities are in m/sample,
                 * instead of m/s. This means that if you have a linear velocity expressed in SI units, you have to call the function as
                 * setReference(desiredPosition, desiredVelocity*dt)
                 */
                virtual void setReference(const Eigen::Vector3d& desiredPosition,
                                  const Eigen::Vector3d& desiredVelocity);
                virtual void setReference(const KDL::Vector& desiredPosition,
                                  const KDL::Vector& desiredVelocity);


                /**
                 * @brief getReference returns the CoM task reference
                 * @return the CoM task reference \f$R^3\f$ vector describing the actual
                 * CoM position in the world coordinate frame
                 */
                virtual const Eigen::Vector3d& getReference() const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the CoM task.
                 * @param desiredPosition the \f$R^{3}\f$ vector describing the desired position of the CoM
                 * in the world coordinate frame.
                 * @param desireVelocity is a \f$R^{3}\f$ twist describing the desired trajectory velocity,
                 * and it represents a feed-forward term in the task computation
                 */
                virtual void getReference(Eigen::Vector3d& desiredPosition,
                                  Eigen::Vector3d& desiredVelocity) const;


                /**
                 * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
                 * next update()
                 * @return internal velcity reference
                 */
                const Eigen::Vector3d& getCachedVelocityReference() const;


                /**
                 * @brief getActualPosition returns the CoM actual position. You need to call _update(x) for the position to change
                 * @return the \f$R^{3}\f$ vector describing the actual CoM position in the world coordinate frame
                 */
                const Eigen::Vector3d& getActualPosition() const;

                /**
                 * @brief getBaseLink an utility function that always returns "world"
                 * @return "world"
                 */
                const std::string& getBaseLink() const;

                /**
                 * @brief getDistalLink an utility function that always
                 * @return
                 */
                const std::string& getDistalLink() const;

                void setLambda(double lambda);

                /**
                 * @brief getError returns the position error between actual and reference positions
                 * @return a \f$R^{3}\f$ vector describing cartesian error between actual and reference position
                 */
                const Eigen::Vector3d& getError() const;

                /**
                 * @brief reset set as actual Cartesian reference the actual pose
                 * @return
                 */
                bool reset();
                
                virtual void _log(XBot::MatLogger2::Ptr logger);

                static bool isCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                static OpenSoT::tasks::velocity::CoM::Ptr asCoM(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
            };
            


        }
    }
 }

#endif
