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


#ifndef __CONSTRAINT_FLUSHER_H__
#define __CONSTRAINT_FLUSHER_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/logger/flushers/Flusher.h>

#include <boost/shared_ptr.hpp>
#include <ostream>

namespace OpenSoT 
{
    namespace flushers 
    {
        class ConstraintFlusher : public Flusher
        {
        protected:
            OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr _constraint;
        public:
            typedef boost::shared_ptr<ConstraintFlusher> Ptr;

            ConstraintFlusher(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint)
                : _constraint(constraint) {}
            virtual ~ConstraintFlusher() {}
        };
    }
}

#endif
