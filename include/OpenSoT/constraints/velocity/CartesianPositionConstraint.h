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
 #include <OpenSoT/tasks/velocity/CoM.h>

#define BOUND_SCALING 0.01

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The CartesianPositionConstraint class implements a constraint of the type
             * \f$A_{\text{Cartesian}}J_{\text{EE}}\dot{q} \leq b_{\text{Cartesian}}-A_{\text{Cartesian}}x_{\text{Cartesian}}\f$
             * where \f$x \in \mathbb{R}^3\f$ is the position of the link controlled by the Cartesian task,
             * \f$A_{\text{Cartesian}} \in \mathbb{R}^{n\times3}\f$ and \f$b_{\text{Cartesian}} \in \mathbb{R}^{n}\f$
             * The matrix \f$A_{\text{Cartesian}}\f$ and the vector \f$b_{\text{Cartesian}}\f$ together represent
             * a plane expressed in the \emph{Cartesian} task frame of reference. We constraint therefore
             * implies we restrict the link movements to be on one side of such plane.
             * In particular, each plane (constraint) can be described as a row of plane coefficients
             * in the matrix \f$D=\left[ A_{\text{Cartesian}} , -b_{\text{Cartesian}}\right]\f$, with
             * \f$D\in\mathbb{R}^{n\times4}\f$.
             * NOTICE It is adviced to apply this constraint only to \emph{Cartesian} tasks that are
             * expressed in the \emph{world} frame of reference, to avoid unexpected behaviors.
             */
            class CartesianPositionConstraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<CartesianPositionConstraint> Ptr;
            private:
                OpenSoT::tasks::velocity::Cartesian::Ptr _cartesianTask;
                OpenSoT::tasks::velocity::CoM::Ptr _comTask;
                Eigen::MatrixXd _A_Cartesian;
                Eigen::VectorXd _b_Cartesian;
                double _boundScaling;
                bool _is_Cartesian;
                Eigen::MatrixXd J;
                Eigen::VectorXd currentPosition;

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
                CartesianPositionConstraint(const Eigen::VectorXd& x,
                                             OpenSoT::tasks::velocity::Cartesian::Ptr cartesianTask,
                                             const Eigen::MatrixXd& A_Cartesian,
                                             const Eigen::VectorXd& b_Cartesian,
                                            const double boundScaling = 1.0);

                CartesianPositionConstraint(const Eigen::VectorXd& x,
                                             OpenSoT::tasks::velocity::CoM::Ptr comTask,
                                             const Eigen::MatrixXd& A_Cartesian,
                                             const Eigen::VectorXd& b_Cartesian,
                                            const double boundScaling = 1.0);

                void update(const Eigen::VectorXd &x);

                /**
                 * @brief getCurrentPosition return the current Cartesian position of the bounded Task
                 * @param current_position a 3x1 position vector
                 */
                void getCurrentPosition(Eigen::VectorXd& current_position);

                /**
                 * @brief setAbCartesian update with new A_Cartesian and b_Cartesian
                 * @param A_Cartesian new matrix nx3 specifying the cartesian limits
                 * @param b_Cartesian new vector of size n specifying the cartesian limits
                 */
                void setAbCartesian(const Eigen::MatrixXd& A_Cartesian, const Eigen::VectorXd& b_Cartesian);

            };
        }
    }
 }

#endif
