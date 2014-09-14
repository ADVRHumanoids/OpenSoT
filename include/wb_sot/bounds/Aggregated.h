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
            public:
                typedef Bounds< yarp::sig::Matrix, yarp::sig::Vector> BoundType;

                enum AggregationPolicy {
                    /** transform equalities Ax = b to inequalities b <= Ax <= b */
                    EQUALITIES_TO_INEQUALITIES = 0x001,
                    /** if enabled, unilateral bounds will be converted to unilateral:
                     *      x <= u becomes -inf <= x <= u, l <= x becomes l <= x <= inf
                     *  if not enabled, bilateral bounds will be converted to unilateral:
                     *      l <= x <= u becomes x <= u && -x <= -l
                     */
                    UNILATERAL_TO_BILATERAL = 0x100
                };

            private:

                std::list< BoundType> _bounds;
                unsigned int _aggregationPolicy;

            public:
                /**
                 * @brief Aggregated
                 * @param bounds a std::list of Bounds
                 */
                Aggregated(const std::list<BoundType> &bounds,
                           const unsigned int x_size,
                           const unsigned int aggregationPolicy =
                                EQUALITIES_TO_INEQUALITIES |
                                UNILATERAL_TO_BILATERAL);

                void update(const yarp::sig::Vector &x);
            };
        }
    }
 }

#endif
