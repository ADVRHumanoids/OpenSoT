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


#ifndef __DATA_FLUSHER_H__
#define __DATA_FLUSHER_H__

#include <OpenSoT/utils/logger/flushers/Flusher.h>
#include <boost/shared_ptr.hpp>
#include <ostream>
#include <sstream>

namespace OpenSoT
{
    namespace flushers
    {
        template <class T>
        class DataFlusher : public Flusher
        {
            unsigned int _size;
            const T* _data;
        public:
            enum { ALL = 0 };

            typedef boost::shared_ptr< DataFlusher<T> > Ptr;

            DataFlusher(const T* data, unsigned int size)
                : _data(data), _size(size) {}

            ~DataFlusher() {}


            std::string toString() const
            {
                std::stringstream ss;
                for(unsigned int i = 0; i < _size; ++i)
                    ss << ", " << _data[i];
                return ss.str();
            }

            /**
             * @brief getSize returns the number of elements logged by this flusher
             * @return the number of elements to be logged
             */
            virtual int getSize() const
            {
                return _size;
            }

            Indices getIndices(int label) const
            {
                return Indices::range(0, _size - 1);
            }
        };
    }
}

#endif
