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

// namespace OpenSoT {
//     /*TODO check constructor on CIK*/
//     class AutoStack 
//     {
//         std::vector<OpenSoT::tasks::Aggregated::TaskPtr> _stack;
//         OpenSoT::consraints::Aggregated::ConstraintPtr _bounds;
//         public:
//             operator std::vector<OpenSoT::Aggregated::TaskPtr>();
//             void update(const yarp::sig::Vector & state);
//     };    
// }


OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task2);

OpenSoT::tasks::Aggregated::Ptr operator+(  const OpenSoT::tasks::Aggregated::Ptr aggregated,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task);


OpenSoT::utils::AutoStack::Ptr operator/(   const OpenSoT::tasks::Aggregated::TaskPtr task1,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task2);

OpenSoT::utils::AutoStack::Ptr operator/(   const OpenSoT::utils::AutoStack::Ptr stack,
                                            const OpenSoT::tasks::Aggregated::TaskPtr task);

OpenSoT::utils::AutoStack::Ptr operator/(   const OpenSoT::utils::AutoStack::Ptr stack1,
                                            const OpenSoT::utils::AutoStack::Ptr stack2);

OpenSoT::tasks::Aggregated::TaskPtr& operator<<(const OpenSoT::tasks::Aggregated::TaskPtr task,
                                                const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

OpenSoT::utils::AutoStack::Ptr& operator<<( const OpenSoT::utils::AutoStack::Ptr stack1,
                                            const OpenSoT::constraints::Aggregated::ConstraintPtr constraint);

#endif