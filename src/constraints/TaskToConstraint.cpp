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
    BilateralConstraint(task->getTaskID(), task->getA(), task->getb(), task->getb()),
    _task(task),
    _err_lb(Eigen::VectorXd::Zero(task->getTaskSize())),
    _err_ub(Eigen::VectorXd::Zero(task->getTaskSize()))
{
    this->generateAll();
}

TaskToConstraint::TaskToConstraint(TaskToConstraint::TaskPtr task, 
                                   const Eigen::VectorXd& err_lb, 
                                   const Eigen::VectorXd& err_ub): 
    BilateralConstraint(task->getTaskID(), task->getA(), task->getb() + err_lb, task->getb() + err_ub),
    _task(task),
    _err_lb(err_lb),
    _err_ub(err_ub)
{
    
    if( ((err_ub-err_lb).array() < 0).any() ){
        throw std::runtime_error("Some components of err_ub are smaller than err_lb!!!");
    }
    
    if( err_lb.size() != task->getA().rows() || err_ub.size() != task->getA().rows() ){
        throw std::runtime_error("Either err_ub or err_lb has wrong dimension!!!");
    }
    
    this->generateAll();
}


void TaskToConstraint::update(const Eigen::VectorXd &q)
{
    _task->update(q);
    this->generateAll();
}

void TaskToConstraint::generateAll() {
    
    _Aineq = _task->getA();
    _bLowerBound = _task->getb() + _err_lb;
    _bUpperBound = _task->getb() + _err_ub;
    

    assert( (_Aineq.rows() == _bLowerBound.rows()) &&
            (_Aineq.rows() == _bUpperBound.rows()));
}
