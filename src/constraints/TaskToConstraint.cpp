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

#include <OpenSoT/constraints/TaskToConstraint.h>

#include <assert.h>
#include <limits>

using namespace OpenSoT::constraints;

TaskToConstraint::TaskToConstraint(TaskPtr task) :
    BilateralConstraint(task->getTaskID(), task->getA(), task->getb(), task->getb()), _task(task)
{
    this->generateAll();
}

void TaskToConstraint::update(const Eigen::VectorXd &q)
{
    assert(q.rows() == _task->getXSize());

    _task->update(q);
    this->generateAll();
}

void TaskToConstraint::generateAll() {
    _Aineq = _task->getA();
    _bLowerBound = _task->getb();
    _bUpperBound = _bLowerBound;

    assert( (_Aineq.rows() == _bLowerBound.rows()) &&
            (_Aineq.rows() == _bUpperBound.rows()));
}
