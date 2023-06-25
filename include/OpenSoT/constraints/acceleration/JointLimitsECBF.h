/*
 * Copyright (C) 2023 Unmanned Systems and Robotics Lab
 * Author: Enrico Mingo Hoffman
 * email:  enricomingo@gmail.com
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

#ifndef __BOUNDS_ACCELERATION_JOINTLIMITS_ECBF_H__
#define __BOUNDS_ACCELERATION_JOINTLIMITS_ECBF_H__

#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>

namespace OpenSoT {
   namespace constraints {
       namespace acceleration{
            /**
             * @brief The JointLimitsECBF class implements Joint Limits using Exponential Control Barrier Functions (ECBF)
             * as in the paper "Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions",
             * by Charles Khazoom, Daniel Gonzalez-Diaz, Yanran Ding, and Sangbae Kim
             */
            class JointLimitsECBF: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimitsECBF> Ptr;
            private:

                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;
                Eigen::VectorXd _jointVelMax;
                Eigen::VectorXd _jointAccMax;

                Eigen::VectorXd __upperBound, _upper_ecbf;
                Eigen::VectorXd __lowerBound, _lower_ecbf;
                Eigen::VectorXd _ll, _uu;

                XBot::ModelInterface& _robot;

                Eigen::VectorXd _q, _qdot;

                GenericConstraint::Ptr _generic_constraint_internal;

                Eigen::VectorXd _a1, _a2, _a3;

                Eigen::VectorXd _ones;

            public:
                /**
                 * @brief JointLimitsECBF constructor
                 * @param robot
                 * @param qddot
                 * @param jointBoundMax max joint limits
                 * @param jointBoundMin min joint limits
                 * @param jointVelMax max joint velocity limits
                 * @param jointAccMax max joint acceleration limits
                 */
                JointLimitsECBF(XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointVelMax,
                            const Eigen::VectorXd &jointAccMax);

                void update(const Eigen::VectorXd& x);

                /**
                 * @brief setAlpha1 gain applied to joint position limits
                 * @param a1
                 */
                void setAlpha1(const Eigen::VectorXd& a1);
                void setAlpha1(const double a1);

                /**
                 * @brief setAlpha2 gain applied to joint position limits
                 * @param a2
                 */
                void setAlpha2(const Eigen::VectorXd& a2);
                void setAlpha2(const double a2);

                /**
                 * @brief setAlpha3 gain applied to joint velocity limits
                 * @param a3
                 */
                void setAlpha3(const Eigen::VectorXd& a3);
                void setAlpha3(const double a3);


            };
       }
   }
}
#endif
