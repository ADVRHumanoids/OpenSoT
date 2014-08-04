/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#include <wb_sot/task.h>
#include <list>

 namespace wb_sot {
    namespace tasks {
        template <unsigned int x_size>
        class Aggregated : public Task<yarp::sig::Matrix, yarp::Sig::Vector, x_size> {
        private:
        public:
            Aggregated(std::list<Task< > >)
        }
    };
 }
