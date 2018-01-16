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

#ifndef __AUTOSTACK_H__
#define __AUTOSTACK_H__

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <XBotInterface/Logger.hpp>
#include <OpenSoT/SubTask.h>

/**
 * @example example_autostack.cpp
 * The AutoStack class allows to use the MOT (Math of Tasks)
 * to define stacks.
 */

namespace OpenSoT {
    /**
     * @brief The AutoStack class eases the managing of a stack of task,
     *        by automatically calling the update() function on all
     *        tasks of the stack. Many operators are defined for the
     *        AutoStack, so that it's possible to concisely write
     *        stacks. Together with OpenSoT::DefaultHumanoidStack
     *        it allows to use the MOT (Math of Tasks) to define stacks, e.g.:
     * AutoStack = (T1 + T2) / (T3 << ConstraintT3 + T4) << Bounds
     *
     * You can see an example in @ref example_autostack.cpp
     */
    class AutoStack 
    {
        public:
        typedef boost::shared_ptr<OpenSoT::AutoStack> Ptr;
        private:
        OpenSoT::solvers::QPOases_sot::Stack _stack;

        OpenSoT::constraints::Aggregated::Ptr _boundsAggregated;

        std::vector<OpenSoT::solvers::QPOases_sot::TaskPtr> flattenTask(
                OpenSoT::solvers::QPOases_sot::TaskPtr task);

        protected:
            AutoStack(const double x_size);

        public:
            AutoStack(OpenSoT::tasks::Aggregated::TaskPtr task);

            AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack);

            AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack,
                      std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds);

            /*AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack,
                      OpenSoT::constraints::Aggregated::ConstraintPtr bound);*/

            void update(const Eigen::VectorXd & state);

            void log(XBot::MatLogger::Ptr logger);

            OpenSoT::solvers::QPOases_sot::Stack& getStack();

            std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>& getBoundsList();

            /**
             * @brief setBoundsAggregationPolicy changes the aggregation policy of the bounds as
             * returned by the getBounds() function. Notice calling this will create a new bounds
             * object (an instance of OpenSoT::constraints::Aggregated), so that the update()
             * function of this AutoStack will then update only the new instance. If you have
             * another instance of those Bounds lying around, you should then manually update that
             * before using it.
             * @param aggregationPolicy the new aggregation policy for this AutoStack's bounds
             */
            void setBoundsAggregationPolicy(const unsigned int aggregationPolicy =
                OpenSoT::constraints::Aggregated::EQUALITIES_TO_INEQUALITIES |
                OpenSoT::constraints::Aggregated::UNILATERAL_TO_BILATERAL);

            OpenSoT::constraints::Aggregated::ConstraintPtr getBounds();

            OpenSoT::solvers::QPOases_sot::TaskPtr getOperationalSpaceTask(const std::string& base_link, const std::string& distal_link);
            OpenSoT::solvers::QPOases_sot::TaskPtr getOperationalSpaceTask(const std::string& task_id);
    };    

/**
 * @brief operator * takes a weight matrix and a task, apply the weight to the task
 * NOTE: the weight is set taking into account the one already set:
 *  Wfinal = W*Woriginal
 * @param W weight matrix
 * @param task a task
 * @return a task ptr
 */
OpenSoT::tasks::Aggregated::TaskPtr operator*(const Eigen::MatrixXd& W,
                                              OpenSoT::tasks::Aggregated::TaskPtr task);

/**
 * @brief operator * takes a weight and a task, apply the weight to the task
 * NOTE: the weight is set taking into account the one already set:
 *  Wfinal = w*Woriginal
 * @param w a weight
 * @param task a task
 * @return  a task ptr
 */
OpenSoT::tasks::Aggregated::TaskPtr operator*(const double w,
                                              OpenSoT::tasks::Aggregated::TaskPtr task);

/**
 * @brief operator * a weight and a task, apply the weight to the task
 * NOTE: the weight is set the same for all the tasks in the aggregate, taking into account the originals
 *  Wfinal = w*Woriginal
 * @param w a weight
 * @param task a task
 * @return a task ptr
 */
OpenSoT::tasks::Aggregated::Ptr operator*(const double w,
                                          OpenSoT::tasks::Aggregated::Ptr task);

/**
 * @brief operator % takes a task and a list of indices, generates a subtask
 * @param task a task pointer
 * @param rowIndices list of indices
 * @return a pointer to a SubTask generated from task with the given indices
 * TODO: this will not work with tasks under the folder torque, in that case another solution
 * should be found
 */
OpenSoT::SubTask::Ptr operator%(const OpenSoT::tasks::Aggregated::TaskPtr task,
                                const std::list<unsigned int> rowIndices);

/**
 * @brief operator + takes two tasks, generates a new Aggregated task
 * @param task1 a task pointer
 * @param task2 a task pointer
 * @return a pointer to a new Aggregated task
 */
OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task2);

/**
 * @brief operator + takes an aggregated and a task, generates a new Aggregated task
 * containing the new task and all the tasks from the old aggregated.
 * It also copies the lambda, the weight matrix and the constraints of the old aggregated.
 * In fact, the new aggregated is a copy of the old aggregated, with a new task,
 * and an adapted weight matrix (its size is increased to take into account the new task)
 * @param aggregated a pointer to an Aggregated task
 * @param task a task pointer pointer
 * @return a pointer to a new Aggregated task
 */
OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task);

/**
 * @brief operator + takes an aggregated and a task, generates a new Aggregated task
 * containing the new task and all the tasks from the old aggregated.
 * It also copies the lambda, the weight matrix and the constraints of the old aggregated.
 * In fact, the new aggregated is a copy of the old aggregated, with a new task,
 * and an adapted weight matrix (its size is increased to take into account the new task)
 * @param task a task pointer pointer
 * @param aggregated a pointer to an Aggregated task
 * @return a pointer to a new Aggregated task
 */
OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated);

/**
 * @brief operator + takes two Aggregated tasks, and creates a new Aggregated task.
 * The way the new Aggregated is created, depends on the two Aggregated tasks that we want to merge.
 * If aggregated1 and aggregated2 hav different lambdas, the new Aggregated will have aggregated1
 * and aggregated2 as tasks.
 * If, on the other side, the two Aggregateds have different Lambdas, a new Aggregated will be created
 * containing ALL the tasks in aggregated1 plus ALL the tasks in aggregated2.
 * In any case, the weight matrix of the new Aggregated will be a block-diagonal matrix where the diagonal
 * blocks are the weight matrices of aggregated1 and aggregated2.
 * The constraints of the Aggregated are the union of the constraints of aggregated1 and aggregated2.
 * (NOTICE that the equality test for the constraints is on pointers, so you can have duplicated constraints
 * if you made multiple instances of the same constraints)
 * @param aggregated1 the first Aggregated task
 * @param aggregated2 the second Aggregated task
 * @return a new Aggregated task containing the tasks in aggregated1 and aggregated2,
 * or containing aggregated1 and aggregated2 as tasks if they have different lambdas in the moment
 * when the + operator is called.
 */
OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated1,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated2);


/**
 * @brief operator / creates a new AutoStack with task1 at the first level and task2 at the second
 * @param task1 a pointer to the first task
 * @param task2 a pointer to the second task
 * @return a new Autostack
 */
OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task2);


/**
 * @brief operator / creates a new AutoStack that is a copy of an autostack with
 * a new task piled at lowest priority.
 * NOTICE that the bounds of the original AutoStack are carried over.
 * @param stack a pointer to an Autostack
 * @param task a pointer to a task
 * @return a new Autostack
 */
OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task);

/**
 * @brief operator / creates a new AutoStack that is a copy of an autostack with
 * a new task piled at highest priority.
 * NOTICE that the bounds of the original AutoStack are carried over.
 * @param task a pointer to a task
 * @param stack a pointer to an Autostack
 * @return a new Autostack
 */
OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                    OpenSoT::AutoStack::Ptr stack);

/**
 * @brief operator / creates a new AutoStack that stacks all tasks
 * a of stack1, followed by all tasks of stack2. That is, the priority of the tasks
 * in stack1 and in stack2 are preserved, and all tasks from stack1 will be of higher
 * priority w.r.t. all tasks in stack2.
 * NOTICE that the bounds of the new AutoStack will be the union of the bounds
 * of stack1 and stack2.
 * (Also NOTICE that the equality test for the bounds is on pointers, so you can have duplicated
 * bounds if you made multiple instances of the same bounds/constraints)
 * @param stack1 a pointer to the first stack
 * @param stack2 a pointer to the second stack
 * @return a new AutoStack
 */
OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack1,
                                    const OpenSoT::AutoStack::Ptr stack2);

/**
 * @brief operator << adds a new constraint to the task specified
 * @param task a pointer to the task
 * @param constraint a pointer to the constraint
 * @return a pointer to the same input task, with a constraint added
 * (NOTICE the task is NOT a copy, it's the input task to which we
 * added a new constraint)
 */
OpenSoT::tasks::Aggregated::TaskPtr operator<<( OpenSoT::tasks::Aggregated::TaskPtr task,
                                                const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

/**
 * @brief operator << adds a new constraint to the task specified
 * @param task a pointer to the task
 * @param constraint a pointer to the constraint
 * @return a pointer to the same input task, with a constraint added
 * (NOTICE the task is NOT a copy, it's the input task to which we
 * added a new constraint)
 */
OpenSoT::tasks::Aggregated::Ptr operator<<( OpenSoT::tasks::Aggregated::Ptr task,
                                            const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

/**
 * @brief operator << adds a new constraint/bound to the stack specified.
 * A unicity test will be made, so that if the stack already has the input
 * bound in the list of bounds, it will not get added.
 * (NOTICE that the equality test for the bounds is on pointers, so you can have duplicated
 * bounds if you made multiple instances of the same bounds/constraints)
 * @param task a pointer to the task
 * @param bound a pointer to the bound/constraint
 * @return a pointer to the same input stack, with a constraint/bound added
 * (NOTICE the stack is NOT a copy, it's the input stack to which we
 * added a new constraint)
 */
OpenSoT::AutoStack::Ptr operator<<( OpenSoT::AutoStack::Ptr stack1,
                                    const OpenSoT::constraints::Aggregated::ConstraintPtr bound);
}
#endif
