/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo Hoffman
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

#ifndef __DEFAULTHUMANOIDSTACK_H__
#define __DEFAULTHUMANOIDSTACK_H__

#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/MinimumVelocity.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <OpenSoT/tasks/velocity/Gaze.h>
#include <XBotInterface/ModelInterface.h>

 namespace OpenSoT {

     class DefaultHumanoidStack
     {
        public:
         typedef std::shared_ptr<DefaultHumanoidStack> Ptr;

         /**
          * @brief DefaultHumanoidStack creates a default set of stacks for a humanoid robot
          * @param model the robot model. It should be updated before creating the DHS (Default Humanoid Stack)
          *              in order to impose good initial references for the tasks
          * @param dT the control time in [s].
          * @param state
          */
         DefaultHumanoidStack(XBot::ModelInterface &model,
                              const double dT,
                              std::string base_link,
                              std::string l_hand, std::string r_hand,
                              std::string l_foot, std::string r_foot,
                              double joint_velocity_limits,
                              const Eigen::VectorXd& state);

         virtual ~DefaultHumanoidStack() {}

         // tasks
         tasks::velocity::Cartesian::Ptr leftArm;
         SubTask::Ptr leftArm_Position;
         SubTask::Ptr leftArm_Orientation;

         tasks::velocity::Cartesian::Ptr rightArm;
         SubTask::Ptr rightArm_Position;
         SubTask::Ptr rightArm_Orientation;

         tasks::velocity::Cartesian::Ptr waist2LeftArm;
         SubTask::Ptr  waist2LeftArm_Position;
         SubTask::Ptr  waist2LeftArm_Orientation;

         tasks::velocity::Cartesian::Ptr waist2RightArm;
         SubTask::Ptr   waist2RightArm_Position;
         SubTask::Ptr   waist2RightArm_Orientation;

         tasks::velocity::Cartesian::Ptr leftLeg;
         SubTask::Ptr   leftLeg_Position;
         SubTask::Ptr   leftLeg_Orientation;

         tasks::velocity::Cartesian::Ptr rightLeg;
         SubTask::Ptr   rightLeg_Position;
         SubTask::Ptr   rightLeg_Orientation;

         tasks::velocity::Cartesian::Ptr right2LeftLeg;

         tasks::velocity::Cartesian::Ptr waist;
         SubTask::Ptr waist_Position;
         SubTask::Ptr waist_Position_XY;
         SubTask::Ptr waist_Position_Z;
         SubTask::Ptr waist_Orientation;

         tasks::velocity::CoM::Ptr com;
         SubTask::Ptr com_XY;
         SubTask::Ptr com_Z;

         tasks::velocity::Gaze::Ptr gaze;
         tasks::velocity::Gaze::Ptr waist2gaze;

         tasks::velocity::MinimumVelocity::Ptr minimumVelocity;
         tasks::velocity::Postural::Ptr postural;


         constraints::velocity::CoMVelocity::Ptr comVelocity;
         constraints::velocity::JointLimits::Ptr jointLimits;
         constraints::velocity::VelocityLimits::Ptr velocityLimits;
     };
 };

#endif
