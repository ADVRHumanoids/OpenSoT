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

#ifndef __BOUNDS_VELOCITY_JOINTLIMITS_AFFINE_H__
#define __BOUNDS_VELOCITY_JOINTLIMITS_AFFINE_H__

#include <OpenSoT/Constraint.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>

 namespace OpenSoT {
    namespace constraints {
        namespace acceleration{ 
            /**
             * @brief The JointLimits class implements bounds on joints positions using internally the
             * GenericConstraint and affine variables
             */
            class JointLimits: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<JointLimits> Ptr;
            private:
                
                Eigen::VectorXd _jointLimitsMin;
                Eigen::VectorXd _jointLimitsMax;

                Eigen::VectorXd __upperBound;
                Eigen::VectorXd __lowerBound;
                
                Eigen::VectorXd _invFunUpperBound;
                Eigen::VectorXd _invFunLowerBound;
                
                Eigen::VectorXd _jointAccMax;

                XBot::ModelInterface& _robot;
                
                Eigen::VectorXd _q, _qdot;
                
                GenericConstraint::Ptr _generic_constraint_internal;

                
            public:
                /**
                 * @brief JointLimits constructor
                 * @param q the configuration of the robot when
                 *          creating the joint limits constraint
                 * @param jointBoundMax upper bounds for joint limits
                 * @param jointBounMin lower bounds for joint limits
                 */
                JointLimits(XBot::ModelInterface& robot,
                            const AffineHelper& qddot,
                            const Eigen::VectorXd &jointBoundMax,
                            const Eigen::VectorXd &jointBoundMin,
                            const Eigen::VectorXd &jointAccMax);


                void update(const Eigen::VectorXd& x);

            };
           }
        }
    
 }

#endif
