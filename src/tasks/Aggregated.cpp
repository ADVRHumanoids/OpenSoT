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
#include <algorithm>
#include <exception>
#include <stdexcept>
#include <assert.h>

using namespace OpenSoT::tasks;
using namespace yarp::math;

Aggregated::Aggregated(const std::list<TaskPtr> tasks,
                       const unsigned int x_size) :
    Task(concatenateTaskIds(tasks),x_size), _tasks(tasks)
{
    assert(tasks.size()>0);

    this->checkSizes();
    /* calling update to generate bounds */
    this->generateAll();

    _W.resize(_A.rows(),_A.rows()); _W.eye();

    _hessianType = this->computeHessianType();
}

Aggregated::Aggregated(TaskPtr task1,
                       TaskPtr task2,
                       const unsigned int x_size) :
Task("(" + task1->getTaskID()+"+"+task2->getTaskID() + ")",x_size)
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
    Task(concatenateTaskIds(tasks),q.size()), _tasks(tasks)
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
    for(std::list< TaskPtr >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        TaskPtr t = *i;
        t->update(x);
    }
    this->generateAll();
}


void Aggregated::checkSizes() {
    for(std::list< TaskPtr >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        TaskPtr t = *i;
        assert(this->getXSize() == t->getXSize());
    }
}


void Aggregated::generateAll() {
    _A.resize(0,_x_size);
    _b.resize(0);
    for(std::list< TaskPtr >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        TaskPtr t = *i;
        _A = yarp::math::pile(_A,t->getWeight()*t->getA());
        _b = yarp::math::cat(_b, t->getWeight()*t->getb());
    }
    generateConstraints();
}

void OpenSoT::tasks::Aggregated::generateConstraints()
{
    int constraintsSize = this->_constraints.size();
    int expectedConstraintsSize = this->_aggregatedConstraints.size() + this->_ownConstraints.size();
    if(constraintsSize >= expectedConstraintsSize)
    {
        if(constraintsSize > expectedConstraintsSize) // checking whether the user really added only constraints
        {
            std::list < ConstraintPtr > orderedConstraints = this->_constraints;
            orderedConstraints.sort();
            this->_aggregatedConstraints.sort();

            std::vector< ConstraintPtr > diffs;
            diffs.resize( constraintsSize + expectedConstraintsSize );

            std::vector< ConstraintPtr >::iterator diffs_end = std::set_symmetric_difference( orderedConstraints.begin(),
                                                                                              orderedConstraints.end(),
                                                                                              _aggregatedConstraints.begin(),
                                                                                              _aggregatedConstraints.end(),
                                                                                              diffs.begin() );
            diffs.resize(diffs_end - diffs.begin());

            // checking the differences between the two lists are only additions
            if(diffs.size() != (constraintsSize - expectedConstraintsSize))
                throw std::runtime_error("ERROR. The constraints list has grown since last update, "
                                         "but the changes are not coherent.\n"
                                         "You probably tried deleting some constraints, or you added "
                                         "to the constraints something which was already in the tasks "
                                         "constraints.\n");
            else // saving the added constraints to the ownConstraints list, in the original order
            {
                for (std::list<ConstraintPtr>::iterator j = _constraints.begin(); j!=_constraints.end(); ++j)
                    for( std::vector< ConstraintPtr >::iterator i = diffs.begin() ; i != diffs.end() ; ++i)
                        if(*i==*j)
                            _ownConstraints.push_back(*i);
            }
        }
        else
        {
            // notice how there might be a case where constraintsSize == expectedConstraintsSize,
            // but the list has been modified. These changes are discarded without notifying the user
            // for efficiency reasons. This is unforgiving and a bad design decision.
            // TODO We should therefore change this in the future.
        }

    } else throw std::runtime_error("ERROR. You can only add constraints through getConstraints, "
                                    "not remove them. Look at the documentation for more details.\n");

    this->generateAggregatedConstraints();

    this->_constraints.clear();
    _constraints.insert(_constraints.end(), _aggregatedConstraints.begin(), _aggregatedConstraints.end());
    _constraints.insert(_constraints.end(), _ownConstraints.begin(), _ownConstraints.end());

}

void OpenSoT::tasks::Aggregated::generateAggregatedConstraints()
{
    _aggregatedConstraints.clear();
    for(std::list< TaskPtr >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        TaskPtr t = *i;
        _aggregatedConstraints.insert(_aggregatedConstraints.end(),
                                      t->getConstraints().begin(),
                                      t->getConstraints().end());
    }
}

OpenSoT::HessianType OpenSoT::tasks::Aggregated::computeHessianType()
{
    if(_tasks.size() == 1)
        return (*_tasks.begin())->getHessianAtype();

    // we make a guess: all tasks are HST_POSDEF
    bool allZero = true;
    for(std::list< TaskPtr >::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        TaskPtr t = *i;
        // if at least a task has HST_UNKNOWN, propagate that
        if(t->getHessianAtype() == HST_UNKNOWN) return HST_UNKNOWN;
        if(t->getHessianAtype() != HST_ZERO) allZero = false;
        if(t->getHessianAtype() == HST_POSDEF) return HST_POSDEF;
    }

    if(allZero)
        return HST_ZERO;

    // we assume an hessian type HST_SEMIDEF
    return HST_SEMIDEF;
}

const std::string Aggregated::concatenateTaskIds(const std::list<TaskPtr> tasks) {
    std::string concatenatedId;
    int taskSize = tasks.size();
    for(std::list<TaskPtr>::const_iterator i = tasks.begin(); i != tasks.end(); ++i) {
        concatenatedId += (*i)->getTaskID();
        if(--taskSize > 0)
            concatenatedId += "+";
    }

    if(tasks.size() > 1)
        concatenatedId = "(" + concatenatedId + ")";

    return concatenatedId;
}

void Aggregated::setLambda(double lambda)
{
    if(lambda >= 0.0)
    {
        _lambda = lambda;
        for(std::list<TaskPtr>::const_iterator i = _tasks.begin(); i != _tasks.end(); ++i)
        {
            TaskPtr t = *i;
            t->setLambda(_lambda);
        }
    }
}

bool OpenSoT::tasks::Aggregated::isAggregated(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<OpenSoT::tasks::Aggregated>(task);
}
