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

#ifndef __BOUNDS_TORQUE_JOINTLIMITS_H__
#define __BOUNDS_TORQUE_JOINTLIMITS_H__

 #include <OpenSoT/Constraint.h>
 #include <Eigen/Dense>
#include <XBotInterface/ModelInterface.h>


 namespace OpenSoT {
    namespace constraints {
        namespace torque {
            /**
             * @brief The JointLimits class implements bounds on joints positions
             */
            class JointLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef std::shared_ptr<JointLimits> Ptr;
            private:
                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;
                Eigen::VectorXd _q;
                Eigen::VectorXd _qdot;
                const XBot::ModelInterface& _model;
                Eigen::VectorXd _k, _d;


            public:
                /**
                 * @brief JointLimits constructor
                 * @param q the configuration of the robot when
                 *          creating the joint limits constraint
                 * @param jointBoundMax upper bounds for joint limits
                 * @param jointBounMin lower bounds for joint limits
                 */
                JointLimits(const Eigen::VectorXd &q,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const XBot::ModelInterface& model);

                void update(const Eigen::VectorXd &x);
                void setGains(double k, double d);
                void setGains(const Eigen::VectorXd& k, const Eigen::VectorXd& d);
                void getGains(Eigen::VectorXd& k, Eigen::VectorXd& d) const;

            };
        }
    }
 }

#endif
