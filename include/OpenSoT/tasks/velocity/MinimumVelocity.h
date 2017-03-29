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

#ifndef __TASKS_VELOCITY_MINIMUMVELOCITY_H__
#define __TASKS_VELOCITY_MINIMUMVELOCITY_H__

 #include <OpenSoT/Task.h>
 #include <Eigen/Dense>
 #include <kdl/frames.hpp>

/**
 * @example example_MinimumVelocity.cpp
 * The MinimumVelocity class implements a task that tries to minimize joints velocities.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The MinimumVelocity class implements a task that tries to minimize joints velocities.
             * Notice that you can implement task space minimum velocity tasks by setting lambda to 0 in the corresponding tasks, i.e.:
                MinimumCOMVelocity
                MinimumCartesianVelocity
             * You can see an example of it in @ref example_MinimumVelocity.cpp
             */
            class MinimumVelocity : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<MinimumVelocity> Ptr;
            protected:

            public:

                MinimumVelocity(const int& x_size);

                ~MinimumVelocity();

                void _update(const Eigen::VectorXd& x);
            };
        }
    }
 }

#endif
