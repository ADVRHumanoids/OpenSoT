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
            CapturePointConstraint(const Eigen::VectorXd& x,
                                   OpenSoT::tasks::velocity::CoM::Ptr comTask,
                                   const Eigen::MatrixXd& A_Cartesian,
                                   const Eigen::VectorXd& b_Cartesian,
                                   const double dT,
                                   const double boundScaling = 1.0);
            void update(const Eigen::VectorXd &x);
        };
       }
   }
}

#endif
