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
 #include <xbot2_interface/xbotinterface2.h>
 #include <kdl/frames.hpp>
 #include <Eigen/Dense>

 #define WORLD_FRAME_NAME "world"

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Cartesian class implements a task that tries to impose a pose (position and orientation)
             * of a distal link w.r.t. a base link. The reference for the cartesian task is set in base link
             * coordinate frame, or in world if the base link name is set to "world".
             * When relative Cartesian task is required,for example from link A to link B,
             * the relative velocity considered is the the one of B respect to A expressed
             * in A.
             * The Cartesian Task is implemented so that
             * \f$A={}^\text{base}J_\text{distal}\f$
             * and
             * \f$b=K_p*e+\frac{1}{\lambda}\xi_d\f$
             *
             * You can see an example in @ref example_cartesian.cpp
             */
            class Cartesian : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
                
            public:
                
                typedef std::shared_ptr<Cartesian> Ptr;
                
            protected:
                
                virtual void _log(XBot::MatLogger2::Ptr logger);
                
                XBot::ModelInterface& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                Eigen::Affine3d _actualPose;
                Eigen::Affine3d _desiredPose;
                Eigen::Vector6d _desiredTwist, _desiredTwistRef;

                bool _base_link_is_world;

                void update_b();

                double _orientationErrorGain;

                bool _is_initialized;

                Eigen::Vector6d _error;

                Eigen::Affine3d _tmpMatrix, _tmpMatrix2;

                /**
                 * @brief _base_T_distal is used to change new distal link!
                 */
                Eigen::Affine3d _base_T_distal;

                Eigen::Vector3d positionError;
                Eigen::Vector3d orientationError;

                bool _rotate_to_local;
                bool _velocity_refs_are_local;

                Eigen::MatrixXd _tmp_A;
                Eigen::VectorXd _tmp_b;

                Eigen::Vector6d _tmp_twist;

            public:
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
                          XBot::ModelInterface &robot,
                          const std::string& distal_link,
                          const std::string& base_link);

                ~Cartesian();

                virtual void _update();

                /**
                 * @brief setReference sets a new reference for the Cartesian task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function.
                 * It also assumes a null desired velocity at the desired pose, meaning we are trying to achieve a regulation task.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 */
                void setReference(const Eigen::Affine3d& desiredPose);
                void setReference(const Eigen::Matrix4d& desiredPose);
                void setReference(const KDL::Frame& desiredPose);

                /**
                 * @brief setReference sets a new reference for the Cartesian task.
                 * It causes the task error to be recomputed immediately, without the need to call the _update(x) function
                 * Notice how the setReference(desiredPose, desiredTwist) needs to be called before each _update(x) of the Cartesian task,
                 * since THE _update() RESETS THE FEED-FORWARD VELOCITY TERM for safety reasons.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 * @param desireTwist is a \f$R^{6}\f$ twist describing the desired trajectory velocity, and it represents
                 * a feed-forward term in the cartesian task computation. NOTICE how the velocities are in units/sample,
                 * instead of units/s. This means that if you have a twist expressed in SI units, you have to call the function as
                 * setReference(desiredPose, desiredTwist*dt)
                 */
                void setReference(const Eigen::Affine3d& desiredPose,
                                  const Eigen::Vector6d& desiredTwist);
                void setReference(const Eigen::Matrix4d& desiredPose,
                                  const Eigen::Vector6d& desiredTwist);
                void setReference(const KDL::Frame& desiredPose,
                                  const KDL::Twist& desiredTwist);

                /**
                 * @brief setVelocityLocalReference permits to set velocity expressed in local (ee) distal frame
                 * @param desireTwist is a \f$R^{6}\f$ twist describing the desired trajectory velocity, and it represents
                 * a feed-forward term in the cartesian task computation. NOTICE how the velocities are in units/sample,
                 * instead of units/s. This means that if you have a twist expressed in SI units, you have to call the function as
                 * setVelocityLocalReference(desiredTwist*dt)
                 */
                void setVelocityLocalReference(const Eigen::Vector6d& desiredTwist);

                /**
                 * @brief getReference returns the Cartesian task reference
                 * @return the Cartesian task reference \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 */
                void getReference(Eigen::Affine3d& desiredPose) const;
                const Eigen::Matrix4d& getReference() const;
                void getReference(KDL::Frame& desiredPose) const;

                /**
                 * @brief getReference gets the current reference and feed-forward velocity for the Cartesian task.
                 * @param desiredPose the \f$R^{4x4}\f$ homogeneous transform matrix describing the desired pose
                 * for the distal_link in the base_link frame of reference.
                 * @param desireVelocity is a \f$R^{6}\f$ twist describing the desired trajectory velocity, and it represents
                 * a feed-forward term in the cartesian task computation
                 */
                void getReference(Eigen::Affine3d& desiredPose,
                                  Eigen::Vector6d& desiredTwist) const;
                void getReference(Eigen::Matrix4d& desiredPose,
                                  Eigen::Vector6d& desiredTwist) const;
                void getReference(KDL::Frame& desiredPose,
                                  KDL::Vector& desiredTwist) const;


                /**
                 * @brief getActualPose returns the distal_link actual pose. You need to call _update(x) for the actual pose to change
                 * @return the \f$R^{4x4}\f$ homogeneous transform matrix describing the actual pose
                 * for the distal_link in the base_link frame of reference.
                 */
                void getActualPose(Eigen::Affine3d& actual_pose) const;
                const Eigen::Matrix4d& getActualPose() const;
                void getActualPose(KDL::Frame& actual_pose) const;

                /**
                 * @brief getCachedVelocityReference can be used to get Velocity reference after update(), it will reset
                 * next update()
                 * @return internal velcity reference
                 */
                const Eigen::Vector6d& getCachedVelocityReference() const;
                
                void setOrientationErrorGain(const double& orientationErrorGain);
                const double getOrientationErrorGain() const;

                const std::string& getDistalLink() const;
                const std::string& getBaseLink() const;
                const bool baseLinkIsWorld() const;

                virtual void setLambda(double lambda);

                /**
                 * @brief getError returns the 6d cartesian error (position and orientation) between actual and reference pose
                 * @return a \f$R^{6}\f$ vector describing cartesian error between actual and reference pose
                 */
                const Eigen::Vector6d& getError() const;

                /**
                 * @brief setBaseLink change the base link of the task
                 * @param base_link the new base link
                 * @return false if the base link does not exists
                 */
                bool setBaseLink(const std::string& base_link);
                
                /**
                 * @brief Changes the distal link of the task. It also resets the reference according to the current robot pose.
                 * @param distal_link the new distal link
                 * @return false if the distal link does not exists
                 */
                bool setDistalLink(const std::string& distal_link);

                /**
                 * @brief reset set as actual Cartesian reference the actual pose
                 * @return
                 */
                virtual bool reset();

                /**
                 * @brief rotateToLocal rotates both Jacobian and references to local (ee) distal frame, this is mostly used for local subtasks
                 * @param rotate_to_local default is false
                 */
                void rotateToLocal(const bool rotate_to_local);
                
                static bool isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                static OpenSoT::tasks::velocity::Cartesian::Ptr asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

            };
        }
    }
 }

#endif
