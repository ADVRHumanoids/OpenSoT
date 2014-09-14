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
            class Cartesian : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            private:
                iDynUtils& _robot;

                std::string _distal_link;
                std::string _base_link;

                int _distal_link_index;
                int _base_link_index;

                yarp::sig::Matrix _actualPose;
                yarp::sig::Matrix _desiredPose;

                void update_b();

            public:

                yarp::sig::Vector positionError;
                yarp::sig::Vector orientationError;

                /*********** TASK PARAMETERS ************/

                double orientationErrorGain;

                /****************************************/


                Cartesian(std::string task_id,
                          const yarp::sig::Vector& x,
                          iDynUtils &robot,
                          std::string distal_link,
                          std::string base_link);

                ~Cartesian();

                void update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Matrix& desiredPose);

                void setOrientationErrorGain(const double& orientationErrorGain);
            };
        }
    }
 }

#endif
