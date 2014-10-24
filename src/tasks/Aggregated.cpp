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

#include <OpenSoT/tasks/Aggregated.h>
#include <yarp/math/Math.h>
#include <assert.h>

using namespace OpenSoT::tasks;
using namespace yarp::math;

Aggregated::Aggregated(const std::list<TaskPtr> tasks,
                       const unsigned int x_size) :
    Task(std::string("aggregated"),x_size), _tasks(tasks)
{
    this->checkSizes();
    /* calling update to generate bounds */
    this->generateAll();

    _W.resize(_A.rows(),_A.rows()); _W.eye();

    _hessianType = this->computeHessianType();
}

Aggregated::Aggregated(TaskPtr task1,
                       TaskPtr task2,
                       const unsigned int x_size) :
Task(std::string("aggregated"),x_size)
{
    _tasks.push_back(task1);
    _tasks.push_back(task2);

    this->checkSizes();

    /* calling update to generate bounds */
    this->generateAll();

    _W.resize(_A.rows(),_A.rows()); _W.eye();
    _hessianType = this->computeHessianType();
}

Aggregated::Aggregated(const std::list<TaskPtr> tasks,
                       const yarp::sig::Vector& q) :
    Task(std::string("aggregated"),q.size()), _tasks(tasks)
{
    this->checkSizes();
    this->_update(q);

    _W.resize(_A.rows(),_A.rows()); _W.eye();
    _hessianType = this->computeHessianType();
}

Aggregated::~Aggregated()
{
}

void Aggregated::_update(const yarp::sig::Vector& x) {
    for(std::list< boost::shared_ptr<TaskType> >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        boost::shared_ptr<TaskType> t = *i;
        t->update(x);
    }
    this->generateAll();
}


void Aggregated::checkSizes() {
    for(std::list< boost::shared_ptr<TaskType> >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        boost::shared_ptr<TaskType> t = *i;
        assert(this->getXSize() == t->getXSize());
    }
}


void Aggregated::generateAll() {
    this->getConstraints().clear();
    _A.resize(0,_x_size);
    _b.resize(0);
    for(std::list< boost::shared_ptr<TaskType> >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        boost::shared_ptr<TaskType> t = *i;
        _A = yarp::math::pile(_A,t->getWeight()*t->getA());
        _b = yarp::math::cat(_b, t->getWeight()*t->getLambda()*t->getb());
        for(std::list< boost::shared_ptr<ConstraintType> >::iterator j = t->getConstraints().begin();
            j!= t->getConstraints().end(); ++j) {
            this->getConstraints().push_back(*j);
        }
    }
}

OpenSoT::HessianType OpenSoT::tasks::Aggregated::computeHessianType()
{
    if(_tasks.size() == 1)
        return (*_tasks.begin())->getHessianAtype();

    // we make a guess: all tasks are HST_POSDEF
    bool allZero = true;
    for(std::list< boost::shared_ptr<TaskType> >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        boost::shared_ptr<TaskType> t = *i;
        // if at least a task has HST_UNKNOWN, propagate that
        if(t->getHessianAtype() == HST_UNKNOWN) return HST_UNKNOWN;
        if(t->getHessianAtype() != HST_ZERO) allZero = false;
        if(t->getHessianAtype() == HST_POSDEF) return HST_POSDEF;
    }

    if(allZero)
        return HST_ZERO;

    // we assume an hessian type HST_SEMIDEF
    return HST_UNKNOWN;
}
