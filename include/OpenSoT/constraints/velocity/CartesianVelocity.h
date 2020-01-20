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

#ifndef __BOUNDS_VELOCITY_CARTESIANVELOCITY_H__
#define __BOUNDS_VELOCITY_CARTESIANVELOCITY_H__

 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/Cartesian.h>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            class CartesianVelocity: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<CartesianVelocity> Ptr;
            private:
                OpenSoT::tasks::velocity::Cartesian::Ptr _task;
                Eigen::VectorXd _velocityLimits;
                double _dT;

                void generatebBounds();
                void generateAineq();

            public:
                /**
                 * @brief CartesianVelocity constructor
                 * @param velocityLimits a vector of 6 elements describing the maximum twist admissible
                 *                       for the specified Cartesian task, in the task frame of reference.
                 * @param dT the time constant at which we are performing velocity control [s]
                 * @param task a pointer to a Cartesian task. Notice how the task needs to be updated in order
                 *             for the constraint to work
                 */
                CartesianVelocity(const Eigen::VectorXd velocityLimits,
                                  const double dT,
                                  OpenSoT::tasks::velocity::Cartesian::Ptr& task);

                virtual void update(const Eigen::VectorXd &x);

                Eigen::VectorXd getVelocityLimits();
                void setVelocityLimits(const Eigen::VectorXd velocityLimits);
            };
        }
    }
 }

#endif
