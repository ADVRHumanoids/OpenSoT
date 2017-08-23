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
 #include <XBotInterface/ModelInterface.h>
 #include <kdl/frames.hpp>
 #include <Eigen/Dense>

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
            class unicycle : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<unicycle> Ptr;
            protected:
                XBot::ModelInterface& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                Eigen::Affine3d _actualPose;
                Eigen::Affine3d _desiredPose;
                Eigen::VectorXd _desiredTwist;

                bool _base_link_is_world;

                void update_b();

                double _orientationErrorGain;

                bool _is_initialized;

                Eigen::VectorXd _error;

                Eigen::Affine3d _tmpMatrix, _tmpMatrix2;

            public:

                Eigen::VectorXd positionError;
                Eigen::VectorXd orientationError;

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
                unicycle(std::string task_id,
                          const Eigen::VectorXd& x,
                          XBot::ModelInterface &robot,
                          std::string distal_link,
                          std::string base_link);

                ~unicycle();

                void _update(const Eigen::VectorXd& x);

               
                const Eigen::MatrixXd getActualPose() const;
                
				const void getActualPose(KDL::Frame& actual_pose) const;
                
                void setOrientationErrorGain(const double& orientationErrorGain);
                
				const double getOrientationErrorGain() const;

                const std::string getDistalLink() const;
               
				const std::string getBaseLink() const;
                
				const bool baseLinkIsWorld() const;

                void setLambda(double lambda);

                /**
                 * @brief getError returns the 6d cartesian error (position and orientation) between actual and reference pose
                 * @return a \f$R^{6}\f$ vector describing cartesian error between actual and reference pose
                 */
                const Eigen::VectorXd getError() const;

                /**
                 * @brief setBaseLink change the base link of the task
                 * @param base_link the new base link
                 * @return false if the base link does not exists
                 */
                bool setBaseLink(const std::string& base_link);
                
                static bool isCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                static OpenSoT::tasks::velocity::unicycle::Ptr asCartesian(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

            };
        }
    }
 }

#endif
