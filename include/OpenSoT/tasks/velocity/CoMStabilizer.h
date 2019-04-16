/*
 * Copyright (C) 2014 Walkman
 * Authors: Francesco Ruscelli, Enrico Mingo
 * email:  francesco.ruscelli@iit.it, enrico.mingo@iit.it
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

#ifndef __TASKS_VELOCITY_COMSTABILIZER_H__
#define __TASKS_VELOCITY_COMSTABILIZER_H__

#include <OpenSoT/Task.h>
// #include <XBotInterface/ModelInterface.h>
// #include <Eigen/Dense>

#include <compliant_stabilizer/compliantstabilizer.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/Task.h>

#define DEFAULT_samples2ODE 50
#define DEFAULT_freq 10

namespace OpenSoT {
    namespace tasks {
        namespace velocity {

        /**
          * Here we hardcode the base_link and distal_link frames
          */
        #define BASE_LINK_COM "world"
        #define DISTAL_LINK_COM "CoM"
            /**
             * @brief The CoMStabilizer class implements a task that tries to impose a position
             * of the CoM w.r.t. the world frame and ....
             */
            class CoMStabilizer : public CoM {
            public:
                typedef boost::shared_ptr<CoMStabilizer> Ptr;
                
            private:
                
                Eigen::Vector3d _desiredPosition;
                Eigen::Vector3d _desiredVelocity;
                
                CompliantStabilizer _stabilizer;
                
                Eigen::Vector3d _zmp_ref;
                
                Eigen::Vector6d _left_wrench;
                Eigen::Vector6d _right_wrench;
                
                Eigen::Vector3d _l_sole_ref;
                Eigen::Vector3d _r_sole_ref;
                
            public:



                /**
                 * @brief CoMStabilizer
                 * @param x the initial configuration of the robot
                 * @param robot the robot model
                 */
                CoMStabilizer(  const Eigen::VectorXd& x,
                                XBot::ModelInterface& robot,
                                
                                const double sample_time, const double mass,
                                const double ankle_height,
                                const Eigen::Vector2d& foot_size,
                                const double Fzmin,
                                const Eigen::Vector3d& K, const Eigen::Vector3d& C,
                                const Eigen::Vector3d& MaxLims,
                                const Eigen::Vector3d& MinLims,
                                const double samples2ODE=DEFAULT_samples2ODE,
                                const double freq=DEFAULT_freq);


                ~CoMStabilizer();

                virtual void _update(   const Eigen::VectorXd& x);
                
                virtual void setReference(const Eigen::Vector3d& desiredPosition);
                virtual void setReference(const KDL::Vector& desiredPosition);
                virtual void setReference(const Eigen::Vector3d& desiredPosition,
                                  const Eigen::Vector3d& desiredVelocity);
                virtual void setReference(const KDL::Vector& desiredPosition,
                                  const KDL::Vector& desiredVelocity);
                
                virtual Eigen::VectorXd getReference() const;
                virtual void getReference(Eigen::Vector3d& desiredPosition,
                                  Eigen::Vector3d& desiredVelocity) const;
                                  
                void setZMP(Eigen::Vector3d zmp);
                
                void setLeftWrench(Eigen::Vector6d left_wrench);
                void setRightWrench(Eigen::Vector6d right_wrench);
                void setLeftSoleRef(Eigen::Vector3d l_sole_ref);
                void setRightSoleRef(Eigen::Vector3d r_sole_ref);
                
                void setWrench(Eigen::Vector6d left_wrench, Eigen::Vector6d right_wrench);
                void setSoleRef(Eigen::Vector3d l_sole_ref, Eigen::Vector3d r_sole_ref);
               

            };
            


        }
    }
 }

#endif



















