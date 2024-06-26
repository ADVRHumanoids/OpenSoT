/*
 * Copyright (C) 2021 Multidof
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

#ifndef __SUBCONSTRAINT_H__
#define __SUBCONSTRAINT_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Indices.h>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <string>
#include <cassert>
#include <memory>
#include <iterator>

namespace OpenSoT {

class SubConstraint : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
public:

    typedef std::shared_ptr<OpenSoT::SubConstraint> Ptr;

    /**
     * @brief SubConstraint create a SubConstraint object by specifying the father Consatrint through a pointer,
     * and a list of row indices. Notice the row indices start from 0 (c style)
     * @param constrPtr a pointer to the father constraint
     * @param rowIndices a list of indices. The index to the first row is 0.
     */
    SubConstraint(ConstraintPtr constrPtr, const std::list<unsigned int> rowIndices);

    virtual ~SubConstraint(){}

    virtual void update();

protected:
    Indices _subConstraintMap;
    ConstraintPtr _constraintPtr;

    static const std::string _SUBCONSTRAINT_SEPARATION_;

    void generateBound(const Eigen::VectorXd& bound, Eigen::VectorXd& sub_bound);
    void generateConstraint(const Eigen::MatrixXd& A, Eigen::MatrixXd& sub_A);


};

}


#endif
