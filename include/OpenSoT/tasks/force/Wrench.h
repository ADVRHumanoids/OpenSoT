/*
 * Copyright (C) 2017 Cogimon
 * Authors: Enrico Mingo Hoffman
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

#ifndef __TASKS_FORCE_WRENCH_H__
#define __TASKS_FORCE_WRENCH_H__

#include <OpenSoT/Task.h>
#include <Eigen/Dense>


 namespace OpenSoT {
    namespace tasks {
        namespace force {
        /**
             * @brief The Wrench class implements a task that tries to generate a wrench near to the one desired
             */
            class Wrench : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<Wrench> Ptr;
            protected:
                Eigen::VectorXd _x_desired;
                Eigen::VectorXd _x;

                void update_b();

            public:

                Wrench(const Eigen::VectorXd& x);

                ~Wrench(){}

                void _update(const Eigen::VectorXd& x);


                void setReference(const Eigen::VectorXd& x_desired);



                Eigen::VectorXd getReference() const;

                void setLambda(double lambda);

                Eigen::VectorXd getActualWrench();

                Eigen::VectorXd getError();

            };
        }
    }
 }

#endif
