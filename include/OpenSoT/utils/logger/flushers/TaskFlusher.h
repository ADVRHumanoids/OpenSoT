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

#include <boost/shared_ptr.hpp>
#include <OpenSoT/utils/logger/flushers/tasks/Cartesian.h>
#include <OpenSoT/utils/logger/flushers/tasks/CoM.h>

namespace OpenSoT 
{
    namespace flushers
    {
        class TaskFlusher 
        {
        public:
            typedef boost::shared_ptr<TaskFlusher> Ptr;
        }
    }
}