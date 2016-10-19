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

#include <OpenSoT/constraints/BilateralConstraint.h>

#include <assert.h>
#include <limits>
#include <sstream>

using namespace OpenSoT::constraints;

int BilateralConstraint::_constr_count = 0;

BilateralConstraint::BilateralConstraint(const Eigen::MatrixXd &Aineq,
                                         const Eigen::VectorXd &bLowerBound,
                                         const Eigen::VectorXd &bUpperBound) :
    Constraint("bilateral_constr_", Aineq.cols())
{
    std::stringstream tmp; tmp << BilateralConstraint::_constr_count++;
    this->_constraint_id += tmp.str();

    _Aineq = Aineq;
    _bLowerBound = bLowerBound;
    _bUpperBound = bUpperBound;

    assert( (_Aineq.rows() == _bLowerBound.size()) &&
            (_Aineq.rows() == _bUpperBound.size()));
}


BilateralConstraint::BilateralConstraint(const std::string constraintName,
                                         const Eigen::MatrixXd &Aineq,
                                         const Eigen::VectorXd &bLowerBound,
                                         const Eigen::VectorXd &bUpperBound) :
    Constraint(constraintName, Aineq.cols())
{
    _Aineq = Aineq;
    _bLowerBound = bLowerBound;
    _bUpperBound = bUpperBound;

    assert( (_Aineq.rows() == _bLowerBound.rows()) &&
            (_Aineq.rows() == _bUpperBound.rows()));
}

