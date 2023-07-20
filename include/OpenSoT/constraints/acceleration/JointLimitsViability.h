/*
 * Copyright (C) Unmanned Systems and Robotics Lab
 * Author: Andrea Testa, Enrico Mingo Hoffman
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

#ifndef __BOUNDS_ACCELERATION_JOINTLIMITS_VIABILITY_AFFINE_H__
#define __BOUNDS_ACCELERATION_JOINTLIMITS_VIABILITY_AFFINE_H__

#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>

 namespace OpenSoT {
    namespace constraints {
        namespace acceleration{
            /**
             * @brief The JointLimits class implements bounds on joints positions using internally the
             * GenericConstraint and affine variables. It is based on the paper:
             *  "Joint position and velocity bounds in discrete-time acceleration/torque control of robot manipulators", by: Andrea Del Prete
             */
            class JointLimitsViability: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimitsViability> Ptr;
            private:

                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;

                Eigen::VectorXd __upperBound;
                Eigen::VectorXd __lowerBound;


                Eigen::VectorXd _jointAccMax, _jointVelMax;

                XBot::ModelInterface& _robot;

                Eigen::VectorXd _q, _qdot;

                GenericConstraint::Ptr _generic_constraint_internal;


                /**
                 * @brief _dt
                 **/
                double _dt;
                double _p;

                Eigen::VectorXd _ddq_LB_pos, _ddq_UB_pos;
                Eigen::VectorXd _ddq_M1, _ddq_M2, _ddq_M3, _ddq_m2, _ddq_m3;
                void accBoundsFromPosLimits();
                Eigen::VectorXd _ddq_LB_via, _ddq_UB_via;
                Eigen::VectorXd _b_1, _c_1, _ddq_1, _delta_1, _b_2, _c_2, _delta_2;
                void accBoundsFromViability();
                Eigen::VectorXd _ddq_LB_vel, _ddq_UB_vel;
                void computeJointAccBounds();

                void _log(XBot::MatLogger2::Ptr logger);




            public:
                /**
                 * @brief JointLimitsViability
                 * @param robot
                 * @param qddot
                 * @param jointBoundMax max joint limits
                 * @param jointBoundMin min joint limits
                 * @param jointVelMax max joint velocity limits
                 * @param jointAccMax max joint acceleration limits
                 * @param dt discretization time
                 */
                JointLimitsViability(XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointVelMax,
                            const Eigen::VectorXd &jointAccMax,
                            const double dt);


                void update(const Eigen::VectorXd& x);

                /**
                 * @brief setJointAccMax update maximum accelerations
                 * @param jointAccMax
                 */
                void setJointAccMax(const Eigen::VectorXd& jointAccMax);

                /**
                 * @brief setJointVelMax update maximum velocities
                 * @param jointVelMax
                 */
                void setJointVelMax(const Eigen::VectorXd& jointVelMax);

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
