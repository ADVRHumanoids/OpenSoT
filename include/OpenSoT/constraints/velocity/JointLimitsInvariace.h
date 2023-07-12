/*
 * Copyright (C) 2023 Enrico Mingo Hoffman
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

#ifndef __BOUNDS_VELOCITY_JOINTLIMITS_INVARIANCE_H__
#define __BOUNDS_VELOCITY_JOINTLIMITS_INVARIANCE_H__

#include <OpenSoT/Constraint.h>
#include <XBotInterface/ModelInterface.h>
#include <Eigen/Dense>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity{
            /**
             * @brief The JointLimits class implements bounds on joints positions using internally the
             * GenericConstraint and affine variables. It is a variation of the joint limits presented in the paper:
             *  "Invariance control design for nonlinear control affine systems under hard state constraints", by:
             *  J. Wolff and M.Buss
             */
            class JointLimitsInvariance: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimitsInvariance> Ptr;
            private:

                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;

                Eigen::VectorXd _qdot_prev;

                Eigen::VectorXd _jointAccMax;

                const XBot::ModelInterface& _robot;

                Eigen::VectorXd _ub_sup, _ub_inf;
                Eigen::VectorXd _lb_sup, _lb_inf;



                double _dt;

            public:
                /**
                 * @brief JointLimitsInvariance constructor
                 * @param q
                 * @param jointBoundMax max joint limits
                 * @param jointBoundMin min joint limits
                 * @param jointAccMax max joint acceleration
                 * @param robot model
                 * @param dt
                 * @note the model needs to be updated with joint positions and velocities!
                 */
                JointLimitsInvariance(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointAccMax,
                            XBot::ModelInterface& robot,
                            const double dt);


                void update(const Eigen::VectorXd& x);

                /**
                 * @brief setJointAccMax updates joint acceleration limits
                 * @param jointAccMax
                 */
                void setJointAccMax(const Eigen::VectorXd& jointAccMax);


            };
           }
        }

 }

#endif
