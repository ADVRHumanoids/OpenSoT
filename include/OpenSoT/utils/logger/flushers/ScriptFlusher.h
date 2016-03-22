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


#ifndef __SCRIPT_FLUSHER_H__
#define __SCRIPT_FLUSHER_H__

#include <OpenSoT/utils/logger/flushers/FakeFlusher.h>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <limits>

namespace OpenSoT
{
    namespace flushers
    {
        class ScriptFlusher : public FakeFlusher
        {
            std::string _script_name;
            std::list<unsigned int> _cols;
            std::list<boost::any> _args;

        public:
            typedef boost::shared_ptr<ScriptFlusher> Ptr;

            ScriptFlusher(std::string script_name,
                          std::list<unsigned int> cols,
                          unsigned int size,
                          unsigned int indicesOffset = 0,
                          std::list<boost::any> args = std::list<boost::any>());

            ~ScriptFlusher();

            std::string toString() const;
        };
    }
}

#endif
