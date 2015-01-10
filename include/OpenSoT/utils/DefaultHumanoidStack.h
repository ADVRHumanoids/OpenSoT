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
         const tasks::velocity::Cartesian::Ptr rightArm;
         const tasks::velocity::Cartesian::Ptr waist2LeftArm;
         const tasks::velocity::Cartesian::Ptr waist2RightArm;
         const tasks::velocity::Cartesian::Ptr leftLeg;
         const tasks::velocity::Cartesian::Ptr rightLeg;
         const tasks::velocity::Cartesian::Ptr waist;
         const tasks::velocity::CoM::Ptr com;
         const tasks::velocity::MinimumEffort::Ptr minimumEffort;
         const tasks::velocity::Postural::Ptr postural;

         // constraints
         const constraints::velocity::CoMVelocity::Ptr comVelocity;
         const constraints::velocity::ConvexHull::Ptr convexHull;
         const constraints::velocity::JointLimits::Ptr jointLimits;
         const constraints::velocity::VelocityLimits::Ptr velocityLimits;
     };
 };

#endif
