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

#include <yarp/sig/Vector.h>
#include <OpenSoT/utils/Indices.h>

#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <ostream>
#include <utility>

namespace OpenSoT 
{
    namespace flushers
    {
        /**
         * @brief The Flusher interface defines what can write to file.
         */
        class Flusher
        {
        protected:
            yarp::sig::Vector _q_dot;
            std::vector<std::string> _descriptions;
        public:
            typedef boost::shared_ptr<Flusher> Ptr;


            virtual std::string toString() const = 0;
            virtual ~Flusher() {}

            /**
             * @brief getSize returns the number of elements logged by this flusher
             * @return the number of elements to be logged
             */
            virtual int getSize() const = 0;

            virtual Indices getIndices(int label) const = 0;

            std::list<std::string> getDescription();

            std::list<std::string> getDescription(Indices indices);

            bool setDescription(const std::list<std::string> descriptions);

            bool setDescription(const std::list<std::string> descriptions, Indices indices);


            void updateSolution(const yarp::sig::Vector& q_dot);

            /**
             * @brief i generates a Plottable element given a label
             * @param label
             * @return
             */
            std::pair<Flusher*, Indices> i(int label);

            /**
             * @brief i generates a Plottable element given a set of indices
             * @param indices
             * @return
             */
            std::pair<Flusher*, Indices> i(Indices indices);

            /**
             * @brief operator () generates a Plottable element given a label
             * @param label
             * @return
             */
            std::pair<Flusher*, Indices> operator()(int label);

            /**
             * @brief operator () generates a Plottable element given a set of indices
             * @param indices
             * @return
             */
            std::pair<Flusher*, Indices> operator()(Indices indices);
        };
    }
}

std::ostream& operator<<(std::ostream& out, const OpenSoT::flushers::Flusher& flusher);

std::ostream& operator<<(std::ostream& out, const OpenSoT::flushers::Flusher::Ptr& flusher);

#endif
