/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo Hoffman, Alessio Rocchi, Luca Muratore
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it, luca.muratore@iit.it
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

 #ifndef __WALKING_STACK_H__
 #define __WALKING_STACK_H__

 #include <OpenSoT/utils/DefaultHumanoidStack.h>
 #include <OpenSoT/utils/AutoStack.h>

namespace OpenSoT {

     class WalkingStack : public OpenSoT::AutoStack
     {

         enum STANCE_FOOT
         {
             LEFT_FOOT = 1,
             RIGHT_FOOT = -1
         };

     public:
        typedef std::shared_ptr<WalkingStack> Ptr;

         /**
          * @brief WalkingStack creates and tunes a stack for walking
          * @param model the robot model.
          * @param dT the control time in [s].
          * @param state
          */
        WalkingStack(iDynUtils &model,
                          const double dT,
                          const Eigen::VectorXd& state);

        ~WalkingStack() {}

        OpenSoT::DefaultHumanoidStack::Ptr DHS;

        /** \brief eps the suggested value to use as damping when inverting the task jacobians */
        double eps;

        int stance_foot;

        bool switchSupportFoot(const int trj_stance_foot);

     private:
        iDynUtils& _model;

            //robot_model.iDyn3_model.getSensorMeasurement(_ft_index, wrench_in_sensor_frame);

     };
}


 #endif
