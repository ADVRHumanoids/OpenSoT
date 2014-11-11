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
#include <idynutils/idynutils.h>
#include <idynutils/convex_hull.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>



/**
  * @example example_com.cpp
  * The CoM class implements a task that tries to impose a position
  * of the CoM w.r.t. the support foot.
  */
 namespace OpenSoT {
    namespace tasks {
        namespace velocity {

        /**
          * Note that this is the frame where you have to specify the velocity for the DISTAL_LINK_COM.
          * The floating_base_link instead is placed in the idynutils model!
          */
        #define BASE_LINK_COM "world"
        #define DISTAL_LINK_COM "CoM"
            /**
             * @brief The CoM class implements a task that tries to impose a position
             * of the CoM w.r.t. the support foot. Notice how you need to use it with a model with
             * the floating base link set as the support foot.
             * You can see an example in @ref example_com.cpp
             */
            class CoM : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<CoM> Ptr;
            private:
                iDynUtils& _robot;

                yarp::sig::Vector _actualPosition;
                yarp::sig::Vector _desiredPosition;

                void update_b();

            public:

                yarp::sig::Vector positionError;

                /**
                 * @brief CoM
                 * @param x the initial configuration of the robot
                 * @param robot the robot model, with floating base link set on the support foot
                 */
                CoM(const yarp::sig::Vector& x,
                    iDynUtils& robot);

                ~CoM();

                void _update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Vector& desiredPosition);

                yarp::sig::Vector getReference();

                yarp::sig::Vector getActualPosition();

                std::string getBaseLink(){return BASE_LINK_COM;}
                std::string getDistalLink(){return DISTAL_LINK_COM;}

            };
        }
    }
 }

#endif
