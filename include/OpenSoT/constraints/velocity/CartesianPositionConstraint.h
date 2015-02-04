/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __BOUNDS_VELOCITY_CartesianPositionConstraint_H__
#define __BOUNDS_VELOCITY_CartesianPositionConstraint_H__

 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/Cartesian.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/convex_hull.h>
 #include <kdl/frames.hpp>

#define BOUND_SCALING 0.01

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The CartesianPositionConstraint class implements a constraint of the type
             * \f$A_{\text{Cartesian}}J_{\text{EE}}\dot{q} \leq b_{\text{Cartesian}}\f$, where every row in
             * \f$\left[ A_{\text{CH}} , -b_{\text{CH}\right]\f$
            */
            class CartesianPositionConstraint: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<CartesianPositionConstraint> Ptr;
            private:
                OpenSoT::tasks::velocity::Cartesian::Ptr _cartesianTask;
                yarp::sig::Matrix _A_Cartesian;
                yarp::sig::Vector _b_Cartesian;
            public:
                /**
                 * @brief CartesianPositionConstraint
                 * @param x the current joint position of the robot
                 * @param cartesianTask the cartesian task which we want to bound
                 * @param A_Cartesian a matrix nx3 specifying the cartesian limits
                 * @param b_Cartesian a vector of size n specifying the cartesian limits
                 * @param boundScaling a parameter which is inversely proportional to the number of steps
                 * needed to reach the cartesian task limits.
                 */
                CartesianPositionConstraint(const yarp::sig::Vector& x,
                                             OpenSoT::tasks::velocity::Cartesian::Ptr cartesianTask,
                                             const yarp::sig::Matrix& A_Cartesian,
                                             const yarp::sig::Vector& b_Cartesian,
                                            const double boundScaling = 1.0);

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
