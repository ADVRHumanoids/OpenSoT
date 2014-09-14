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

#include <wb_sot/tasks/Aggregated.h>

#include <yarp/math/Math.h>
#include <assert.h>

using namespace wb_sot::tasks;
using namespace yarp::math;

Aggregated::Aggregated(const std::list< TaskType* >& tasks,
                       const unsigned int x_size) :
    Task(std::string("Aggregated"),x_size), _tasks(tasks)
{
    /* calling update to generate bounds */
    update(yarp::sig::Vector(x_size, 0.0));
}

void Aggregated::update(const yarp::sig::Vector& x) {
    this->constraints.clear();
    _A.clear();
    _b.clear();
    for(std::list< TaskType *>::iterator i = _tasks.begin();
        i != _tasks.end(); ++i) {
        (*i)->update(x);
        _A = yarp::math::pile(_A,(*i)->getA());
        _b = yarp::math::cat(_b, (*i)->getb());
        for(std::list< BoundType* >::iterator j = (*i)->getConstraints();
            j!= _tasks.end(); ++j) {
            this->constraints.push_back(*j);
        }
    }
}
