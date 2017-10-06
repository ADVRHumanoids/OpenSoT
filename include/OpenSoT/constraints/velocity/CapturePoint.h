/*
 * Copyright (C) 2017 Cogimon
 * Author: Enrico Mingo Hoffman
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

#ifndef __BOUNDS_VELOCITY_CapturePointConstraint_H__
#define __BOUNDS_VELOCITY_CapturePointConstraint_H__

#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>

namespace OpenSoT {
   namespace constraints {
       namespace velocity {
       /**
         * @brief The CapturePointConstraint class implements a constraint on the capture point:
         *
         *  \f$ w\mathbf{A}_{\text{Cartesian}}\mathbf{J}_{\text{CoM}}\mathbf{\dot{q}} \leq \mathbf{b}_{\text{Cartesian}} -
         *  \mathbf{A}_{\text{Cartesian}}\mathbf{x}_{\text{CoM}} \f$
         *
         * where \f$w = \sqrt{\frac{z_{\text{CoM}}}{g}} \f$
         *
         * The Constraint is based on the CartesianPositionConstraint
         */
        class CapturePointConstraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
        public:
            typedef boost::shared_ptr<CapturePointConstraint> Ptr;
        private:
            CartesianPositionConstraint::Ptr _cartesian_position_cstr;
            double w;
            Eigen::VectorXd com;
            double _dT;


            Eigen::MatrixXd _A_Cartesian;
            Eigen::VectorXd _b_Cartesian;
        public:
            /**
             * @brief CapturePointConstraint implement a constraint on the Capture Point
             * @param x the current joint position of the robot
             * @param comTask a com task
             * @param A_Cartesian a matrix nx2 specifying the cartesian limits
             * @param b_Cartesian a vector of size n specifying the cartesian limits
             * @param dT control loop
             * @param boundScaling a parameter which is inversely proportional to the number of steps
             * needed to reach the cartesian task limits.
             */
            CapturePointConstraint(const Eigen::VectorXd& x,
                                   OpenSoT::tasks::velocity::CoM::Ptr comTask,
                                   const Eigen::MatrixXd& A_Cartesian,
                                   const Eigen::VectorXd& b_Cartesian,
                                   const double dT,
                                   const double boundScaling = 1.0);

            /**
             * @brief setAbCartesian update with new A_Cartesian and b_Cartesian
             * @param A_Cartesian new matrix nx2 specifying the cartesian limits
             * @param b_Cartesian new vector of size n specifying the cartesian limits
             */
            void setAbCartesian(const Eigen::MatrixXd& A_Cartesian, const Eigen::VectorXd& b_Cartesian);

            void update(const Eigen::VectorXd &x);
        };
       }
   }
}

#endif
