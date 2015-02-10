/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#ifndef __VELOCITYALLOCATION_H__
#define __VELOCITYALLOCATION_H__

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/Solver.h>

/**
 * @example example_cartesian.cpp
 * The Cartesian class implements a task that tries to impose a pose (position and orientation)
 * of a distal link w.r.t. a base link.
 */

namespace OpenSoT
{
    /**
     * @brief The VelocityAllocation class takes care of correctly applying
     * velocity bounds to all the tasks in a stack.
     * It is a common problem of hierarchical control schemes, that the highest priority
     * tasks exhaust the system capabilities (e.g., the first tasks  require joint motions
     * that are at the limits of the maximum allower velocity, or saturate the available torque
     * on the actuators), leaving no resources to use for the lowest priority tasks.
     * When applied on a  stack (either an OpenSoT::Solver::Stack or an OpenSoT::AutoStack)
     * it will go through all levels of the stack and add (if already not existing) a velocity
     * bound with velocity limits increasing for each level of the stack.
     * If working on an AutoStack, it will also check that the AutoStack bounds don't already
     * include a velocity bound, and if yes, it will delete it.
     *
     * You can see an example in @ref example_velocity_allocation.cpp
     *
     * Notice in the future this class will be substituted by a generic ResourceAllocation class,
     * so that, regardless of the control method, we can apply a scalable bound depending on the
     * priority of the task.
     */
    class VelocityAllocation
    {
    public:
        /**
         * @brief VelocityAllocation creates a VelocityAllocation object.
         * The constructor will go through all levels of the stack and add
         * (if already not existing) a velocity bound with velocity limits
         * increasing for each level of the stack.
         * If the velocity bounds already exist for a task, it will just update
         * the velocity limits.
         * The highest priority task will get min_velocity velocity bounds,
         * the lowest priority task will get max_velocity velocity bounds,
         * the reamining tasks will have velocity bounds computed as
         * min_velocity + task_index*(max_velocity - min_velocity)/(stack_size-1)
         * @param stack the tasks stack
         * @param dT sample time
         * @param min_velocity the velocity bounds for the maximum priority task
         * @param max_velocity the velocity bounds for the lowest priority task
         */
        VelocityAllocation(OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack& stack,
                           const double dT,
                           const double min_velocity,
                           const double max_velocity);

        /**
         * @brief VelocityAllocation creates a VelocityAllocation object.
         * The constructor will go through all levels of the stack and add
         * (if already not existing) a velocity bound with velocity limits
         * increasing for each level of the stack.
         * If the velocity bounds already exist for a task, it will just update
         * the velocity limits.
         * It will also check that the AutoStack bounds don't already
         * include a velocity bound, and if yes, it will set its velocity limit
         * to max_velocity.
         * The highest priority task will get min_velocity velocity bounds,
         * the lowest priority task will get max_velocity velocity bounds,
         * the reamining tasks will have velocity bounds computed as
         * min_velocity + task_index*(max_velocity - min_velocity)/(stack_size-1)
         * @param autoStack
         * @param dT sample time
         * @param min_velocity the velocity bounds for the maximum priority task
         * @param max_velocity the velocity bounds for the lowest priority task
         */
        VelocityAllocation(OpenSoT::AutoStack::Ptr autoStack,
                           const double dT,
                           const double min_veloicity,
                           const double max_velocity);

    private:
        double _dT;
        double _min_velocity;
        double _max_velocity;

        void processStack(OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack& stack);
        double computeVelocityLimit(const unsigned int taskIndex,
                                    const unsigned int stackSize);
    };
}

#endif
