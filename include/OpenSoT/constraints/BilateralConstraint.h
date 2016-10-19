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

#ifndef __BOUNDS_BILATERALCONSTRAINT_H__
#define __BOUNDS_BILATERALCONSTRAINT_H__

#include <OpenSoT/Constraint.h>

#include <Eigen/Dense>
#include <list>


 namespace OpenSoT {
    namespace constraints {

        /**
         * @brief The BilateralConstraint class implements a constraint of the form
         *        bLowerBound <= Aineq*x <= bUpperbound
         */
        class BilateralConstraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            static int _constr_count;
        public:
            typedef boost::shared_ptr<BilateralConstraint> Ptr;

            /**
             * @brief BilateralConstraint a bilateral constraint
             * @param Aineq constraint matrix. Number of columns must be > 0
             * @param bLowerBound lower bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             * @param bUpperBound upper bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             */
            BilateralConstraint(const Eigen::MatrixXd &Aineq,
                                const Eigen::VectorXd &bLowerBound,
                                const Eigen::VectorXd &bUpperBound);

            /**
             * @brief BilateralConstraint a bilateral constraint
             * @param constraintName the name of the constraint
             * @param Aineq constraint matrix. Number of columns must be > 0
             * @param bLowerBound lower bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             * @param bUpperBound upper bound vector. Number of rows must be the same
             *                    size as the number of rows of Aineq
             */
            BilateralConstraint(const std::string constraintName,
                                const Eigen::MatrixXd &Aineq,
                                const Eigen::VectorXd &bLowerBound,
                                const Eigen::VectorXd &bUpperBound);

        };
    }
 }

#endif
