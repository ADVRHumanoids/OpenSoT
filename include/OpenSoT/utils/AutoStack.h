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

namespace OpenSoT {
    /*TODO check constructor on CIK*/
    class AutoStack 
    {
        public:
        typedef boost::shared_ptr<OpenSoT::AutoStack> Ptr;
        private:
        OpenSoT::solvers::QPOases_sot::Stack _stack;
        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> _bounds;
        public:
            AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack);

            AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack,
                      std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds);

            /*AutoStack(OpenSoT::solvers::QPOases_sot::Stack stack,
                      OpenSoT::constraints::Aggregated::ConstraintPtr bound);*/

            void update(const yarp::sig::Vector & state);

            OpenSoT::solvers::QPOases_sot::Stack& getStack();

            std::list<OpenSoT::constraints::Aggregated::ConstraintPtr>& getBoundsList();

            OpenSoT::constraints::Aggregated::ConstraintPtr getBounds(  const unsigned int aggregationPolicy =
                                                                        OpenSoT::constraints::Aggregated::EQUALITIES_TO_INEQUALITIES |
                                                                        OpenSoT::constraints::Aggregated::UNILATERAL_TO_BILATERAL);
    };    
}


OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task2);


OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task);

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated);

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated1,
                                            const OpenSoT::tasks::Aggregated::Ptr aggregated2);


OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task2);

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack,
                                    const OpenSoT::tasks::Aggregated::TaskPtr task);

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::tasks::Aggregated::TaskPtr task,
                                    OpenSoT::AutoStack::Ptr stack);

OpenSoT::AutoStack::Ptr operator/(  const OpenSoT::AutoStack::Ptr stack1,
                                    const OpenSoT::AutoStack::Ptr stack2);


OpenSoT::tasks::Aggregated::TaskPtr operator<<( OpenSoT::tasks::Aggregated::TaskPtr task,
                                                const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

OpenSoT::AutoStack::Ptr operator<<( OpenSoT::AutoStack::Ptr stack1,
                                    const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

#endif
