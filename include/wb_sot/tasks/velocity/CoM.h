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

 #include <wb_sot/Task.h>
 #include <drc_shared/idynutils.h>
 #include <drc_shared/utils/convex_hull.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
#include <yarp/os/all.h>

 namespace wb_sot {
    namespace tasks {
        namespace velocity {
            class CoM : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            private:
                iDynUtils _robot;

                /**
                 * @brief _support_foot_link_index in fixed base systems,
                 *                                 we want to specify the CoM wrt
                 *                                 a fixed root.
                 */
                int _support_foot_link_index;
                /**
                 * @brief _swing_foot_link_index during single foot support phase,
                 *                               the swing foot is considered like an end-effector
                 */
                int _swing_foot_link_index;

                yarp::sig::Vector _actualPosition;
                yarp::sig::Vector _desiredPosition;

                void update_b();

            public:

                yarp::sig::Vector positionError;

                CoM(const yarp::sig::Vector& x);

                ~CoM();

                void update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Vector& desiredPosition);
            };
        }
    }
 }

#endif
