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

#ifndef __BOUNDS_VELOCITY_AGGREGATED_H__
#define __BOUNDS_VELOCITY_AGGREGATED_H__

#include <wb_sot/Bounds.h>

#include <yarp/sig/all.h>
#include <list>


 namespace wb_sot {
    namespace bounds {
        namespace velocity {

            class Aggregated: public Bounds<yarp::sig::Matrix, yarp::sig::Vector> {

                typedef Bounds< yarp::sig::Matrix, yarp::sig::Vector> BoundType;

            private:

                std::list< BoundType* > _bounds;

            public:
                /**
                 * @brief Aggregated
                 * @param bounds a std::list of Bounds
                 */
                Aggregated(const std::list<BoundType *> &bounds,
                           const unsigned int x_size);

                const yarp::sig::Vector getLowerBound();
                const yarp::sig::Vector getUpperBound();

                const yarp::sig::Matrix getAeq();
                const yarp::sig::Vector getbeq();

                const yarp::sig::Matrix getAineq();
                const yarp::sig::Vector getbLowerBound();
                const yarp::sig::Vector getbUpperBound();

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
