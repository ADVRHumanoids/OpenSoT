/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo Hoffman, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#ifndef __TASK_FLUSHER_CARTESIAN_H__
#define __TASK_FLUSHER_CARTESIAN_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/utils/logger/flushers/TaskFlusher.h>
#include <idynutils/idynutils.h>

namespace OpenSoT
{
    namespace flushers
    {
        namespace tasks
        {
            namespace velocity
            {
                class Cartesian: public TaskFlusher
                {
                public:
                    enum labels
                    {
                        POSITION_ERROR       = 1,
                        ORIENTATION_ERROR    = 2
                    };

                    Cartesian(OpenSoT::tasks::velocity::Cartesian::Ptr cartesian, const iDynUtils& model);

                    std::string toString() const;

                    OpenSoT::Indices getIndices(int label) const;

                    int getSize() const;

                    typedef boost::shared_ptr<Cartesian> Ptr;
                private:
                    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian;
                };
            }
        }
    }
}

#endif
