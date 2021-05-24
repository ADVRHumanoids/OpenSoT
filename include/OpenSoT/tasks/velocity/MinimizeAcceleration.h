/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
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

#ifndef __TASKS_MINIMIZE_ACCELERATION_H__
#define __TASKS_MINIMIZE_ACCELERATION_H__

 #include <OpenSoT/Task.h>
 #include <Eigen/Dense>

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Minimize Acceleration class implements a task that tries to minimize the change in velocity.
             */
            class MinimizeAcceleration : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef std::shared_ptr<MinimizeAcceleration> Ptr;
            protected:
                Eigen::VectorXd _x_before;

            public:

                MinimizeAcceleration(const Eigen::VectorXd& x);

                ~MinimizeAcceleration();

                void setLambda(double lambda);

                void _update(const Eigen::VectorXd& x);
            };
        }
    }
 }

#endif
