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

#ifndef __TASKS_VELOCITY_POSTURAL_H__
#define __TASKS_VELOCITY_POSTURAL_H__

 #include <OpenSoT/Task.h>
 #include <drc_shared/idynutils.h>
 #include <drc_shared/utils/convex_hull.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

/**
 * @example example_postural.cpp
 * The Postural class implements a task that tries to bring the robust posture to a reference posture.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Postural class implements a task that tries to bring the robust posture to a reference posture.
             * You can see an example of it in @ref example_postural.cpp
             */
            class Postural : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<Postural> Ptr;
            protected:
                yarp::sig::Vector _x_desired;
                yarp::sig::Vector _x;

                void update_b();

            public:

                Postural(const yarp::sig::Vector& x);

                ~Postural();

                void _update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Vector& x_desired);
            };
        }
    }
 }

#endif
