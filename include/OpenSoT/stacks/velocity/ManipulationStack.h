/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Luca Muratore, Enrico Mingo Hoffman
 * email:  alessio.rocchi@iit.it, luca.muratore@iit.it, enrico.mingo@iit.it
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

 #ifndef __MANIPULATION_STACK_H__
 #define __MANIPULATION_STACK_H__

 #include <OpenSoT/utils/DefaultHumanoidStack.h>
 #include <OpenSoT/utils/AutoStack.h>

namespace OpenSoT {

     class ManipulationStack : public OpenSoT::AutoStack, public OpenSoT::DefaultHumanoidStack
     {
     public:
        typedef boost::shared_ptr<ManipulationStack> Ptr;

         /**
          * @brief MainpulationStack creates and tunes a stack for manipulation
          * @param model the robot model. It should be updated before creating the stack
          *              in order to impose good initial references for the tasks. The stack
                         will modify the model by setting the world on the left foot (with a identify transform),
                         and both the anchor and the floating base link on the left foot
          * @param dT the control time in [s].
          * @param state
          */
        ManipulationStack(iDynUtils &model,
                          const double dT,
                          const yarp::sig::Vector& state);

        ~ManipulationStack() {};

        /** \brief eps the suggested value to use as damping when inverting the task jacobians */
        double eps;
     };
}


 #endif
