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


#ifndef __FAKE_FLUSHER_H__
#define __FAKE_FLUSHER_H__

#include <OpenSoT/utils/logger/flushers/Flusher.h>
#include <boost/shared_ptr.hpp>

namespace OpenSoT
{
    namespace flushers
    {
        class FakeFlusher : public Flusher
        {
            unsigned int _size;
        public:
            typedef boost::shared_ptr<FakeFlusher> Ptr;

            FakeFlusher(unsigned int size);

            ~FakeFlusher();

            std::string toString() const;

            /**
             * @brief getSize returns the number of elements logged by this flusher
             * @return the number of elements to be logged
             */
            int getSize() const;

            Indices getIndices(int label) const;
        };
    }
}

#endif
