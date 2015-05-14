/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
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

#include <OpenSoT/OpenSoT.h>

 namespace OpenSoT {

     class DefaultHumanoidStack
     {
        public:

         DefaultHumanoidStack(iDynUtils &model,
                              const double dT,
                              const yarp::sig::Vector& state);

         virtual ~DefaultHumanoidStack() {}

         // tasks
         const tasks::velocity::Cartesian::Ptr leftArm;
         const SubTask::Ptr leftArm_Position;
         const SubTask::Ptr leftArm_Orientation;

         const tasks::velocity::Cartesian::Ptr rightArm;
         const SubTask::Ptr rightArm_Position;
         const SubTask::Ptr rightArm_Orientation;

         const tasks::velocity::Cartesian::Ptr waist2LeftArm;
         const SubTask::Ptr  waist2LeftArm_Position;
         const SubTask::Ptr  waist2LeftArm_Orientation;

         const tasks::velocity::Cartesian::Ptr waist2RightArm;
         const SubTask::Ptr   waist2RightArm_Position;
         const SubTask::Ptr   waist2RightArm_Orientation;

         const tasks::velocity::Cartesian::Ptr leftLeg;
         const SubTask::Ptr   leftLeg_Position;
         const SubTask::Ptr   leftLeg_Orientation;

         const tasks::velocity::Cartesian::Ptr rightLeg;
         const SubTask::Ptr   rightLeg_Position;
         const SubTask::Ptr   rightLeg_Orientation;

         const tasks::velocity::Cartesian::Ptr waist;
         const SubTask::Ptr waist_Position;
         const SubTask::Ptr waist_Position_XY;
         const SubTask::Ptr waist_Position_Z;
         const SubTask::Ptr waist_Orientation;

         const tasks::velocity::CoM::Ptr com;
         const SubTask::Ptr com_XY;
         const SubTask::Ptr com_Z;

         const tasks::velocity::MinimumEffort::Ptr minimumEffort;
         const tasks::velocity::MinimumVelocity::Ptr minimumVelocity;
         const tasks::velocity::Postural::Ptr postural;


         // constraints
         const constraints::velocity::CoMVelocity::Ptr comVelocity;
         const constraints::velocity::ConvexHull::Ptr convexHull;
         const constraints::velocity::JointLimits::Ptr jointLimits;
         const constraints::velocity::VelocityLimits::Ptr velocityLimits;
     };
 };

#endif
