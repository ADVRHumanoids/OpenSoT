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

#ifndef __ROBOT_FLUSHERH__
#define __ROBOT_FLUSHERH__

#include <OpenSoT/utils/logger/flushers/Flusher.h>
#include <idynutils/idynutils.h>
#include <idynutils/RobotUtils.h>

namespace OpenSoT
{
    class L;

    namespace flushers
    {
        class RobotFlusher: public Flusher
        {
        public:
            enum labels
            {
                Q       = 1,
                DQ      = 2,
                TAU     = 4
            };

            RobotFlusher(RobotUtils& robot);

            std::string toString() const;

            OpenSoT::Indices getIndices(int label) const;

            int getSize() const;

            void defaultPlot(L&);

            typedef boost::shared_ptr<RobotFlusher> Ptr;
        private:
            RobotUtils& _robot;
            int _nDoFs;
        };
    }
}

#endif
