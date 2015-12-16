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

#ifndef __FLUSHER_H__
#define __FLUSHER_H__


#include <boost/shared_ptr.hpp>
#include <OpenSoT/utils/Indices.h>

namespace OpenSoT 
{
    namespace flushers
    {
        class Flusher
        {
        protected:
            yarp::sig::Vector _q_dot;
        public:
            typedef boost::shared_ptr<Flusher> Ptr;


            virtual std::string toString() const = 0;
            virtual ~Flusher() {}

            /**
             * @brief getSize returns the number of elements logged by this flusher
             * @return the number of elements to be logged
             */
            virtual int getSize() const { return 0; }

            virtual Indices getIndices(int label) const = 0;

            void updateSolution(const yarp::sig::Vector& q_dot)
            {
                _q_dot = q_dot;
            }
        };
    }
}

#endif
