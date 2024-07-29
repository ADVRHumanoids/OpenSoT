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

#ifndef __BOUNDS_ACCELERATION_JOINTLIMITS_PSAP_H__
#define __BOUNDS_ACCELERATION_JOINTLIMITS_PSAP_H__

#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>

namespace OpenSoT {
   namespace constraints {
       namespace acceleration{
            /**
             * @brief The JointLimitsPSAP class implements Joint Limits using the P-Step Ahead Predictor (PSAP) method
             * as in the paper "The Enhanced Compact QP Method for Redundant Manipulators Using Practical Inequality Constraints",
             * by Ki Cheol Park, Pyung Hun Chang, and Seung Ho Kim
             * @note Constraints are implemented inheriting from Constraint base class
             */
            class JointLimitsPSAP: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimitsPSAP> Ptr;
            private:
                Eigen::VectorXd _jointLimitsMin, _jointLimitsMax;
                Eigen::VectorXd _jointVelMax;
                Eigen::VectorXd _jointAccMax;
                Eigen::VectorXd _pmin, _pmax;
                Eigen::VectorXd _vmin, _vmax;
                Eigen::VectorXd _zeros;
                Eigen::VectorXd __upperBound, __lowerBound;
                XBot::ModelInterface& _robot;
                Eigen::VectorXd _q, _qdot;
                GenericConstraint::Ptr _generic_constraint_internal;
                double _dt, _p;

            public:
                /**
                 * @brief JointLimitsPSAP constructor
                 * @param robot
                 * @param qddot
                 * @param jointBoundMax max joint limits
                 * @param jointBoundMin min joint limits
                 * @param jointVelMax max joint velocity limits
                 * @param jointAccMax max joint acceleration limits
                 * @param dt discretization time
                 */
                JointLimitsPSAP(XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointVelMax,
                            const Eigen::VectorXd &jointAccMax,
                            const double dt);

                /**
                 * @brief update method from coonstraint base class to be implemented
                 * @param x
                 */
                void update();

                /**
                 * @brief setPStepAheadPredictor
                 * @param p step predictor coefficient >= 1
                 * @return false if p <1
                 */
                bool setPStepAheadPredictor(const double p);


            };
       }
   }
}
#endif
