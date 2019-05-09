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
             * of the CoM w.r.t. the world frame. Moreover, it takes the CoP and the desired ZMP position and provides a
///             COM motion correction to stabilize the robot.
             * NOTICE: we assume that the measured wrenches are the ones that the world exert on the robot (z forces are positive).
             * If available measured wrenches are the opposite, please set param invertFTSensors to TRUE
             */
            class CoMStabilizer : public CoM {
            public:
                typedef boost::shared_ptr<CoMStabilizer> Ptr;
                
            private:
                
                Eigen::Vector3d _desiredPosition;
                Eigen::Vector3d _desiredVelocity;
                
                CompliantStabilizer _stabilizer;
                
                Eigen::Vector2d _zmp_ref;
                
                Eigen::Vector6d _left_wrench;
                Eigen::Vector6d _right_wrench;
                
                Eigen::Vector3d _l_sole_ref;
                Eigen::Vector3d _r_sole_ref;
                
                XBot::ForceTorqueSensor::ConstPtr _ft_sensor_l_sole;
                XBot::ForceTorqueSensor::ConstPtr _ft_sensor_r_sole;
                
                bool _invertFTSensors; //TODO take out
                
            public:



                /**
                 * @brief CoMStabilizer
                 * @param x the initial configuration of the robot
                 * @param robot the robot model
                 * @param l_sole pose of the left sole
                 * @param r_sole pose of the right sole
                 * @param ft_sensor_l_sole pointer to force/torque sensor of the left foot
                 * @param ft_sensor_r_sole pointer to force/torque sensor of the right foot
                 * @param sample_time
                 * @param mass
                 * @param ankle_height
                 * @param foot_size
                 * @param Fzmin
                 * @param K
                 * @param C
                 * @param MaxLims
                 * @param MinLims
                 * @param invertFTSensors set true if the measured wrenches are the ones that the robot exert on the world  (z forces are negative).
                 * @param samples2ODE
                 * @param freq 
                 */
                CoMStabilizer(  const Eigen::VectorXd& x,
                                XBot::ModelInterface& robot,
                                
                                Eigen::Affine3d l_sole,
                                Eigen::Affine3d r_sole,
                               
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor_l_sole,
                                XBot::ForceTorqueSensor::ConstPtr ft_sensor_r_sole,
                                
                                const double sample_time, const double mass,
                                const double ankle_height,
                                const Eigen::Vector2d& foot_size,
                                const double Fzmin,
                                const Eigen::Vector3d& K, const Eigen::Vector3d& C,
                                const Eigen::Vector3d& MaxLims,
                                const Eigen::Vector3d& MinLims,
                                const bool invertFTSensors=false,
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
                                  
                void setZMP(Eigen::Vector2d zmp);
                
//                 void setLeftWrench(Eigen::Vector6d left_wrench);
//                 void setRightWrench(Eigen::Vector6d right_wrench);
                void setLeftSoleRef(Eigen::Affine3d l_sole_ref);
                void setRightSoleRef(Eigen::Affine3d r_sole_ref);
                
//                 void setWrench(Eigen::Vector6d left_wrench, Eigen::Vector6d right_wrench);
                void setSoleRef(Eigen::Affine3d l_sole_ref, Eigen::Affine3d r_sole_ref);
              
                
               virtual void _log(XBot::MatLogger::Ptr logger);

                const CompliantStabilizer& getStabilizer();


            };
            


        }
    }
 }

#endif



















