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

#include <wb_sot/Bounds.h>

#include <yarp/sig/all.h>
#include <list>


 namespace wb_sot {
    namespace bounds {
        template <unsigned int x_size>
        class Aggregated: public Bounds<yarp::sig::Matrix, yarp::sig::Vector, x_size> {

            typedef Bounds< yarp::sig::Matrix, yarp::sig::Vector, x_size> BoundType;

        private:

            std::list< Bounds<yarp::sig::Matrix, yarp::sig::Vector, x_size> > _bounds;

            yarp::sig::Vector _q;

            yarp::sig::Vector _upperBound;
            yarp::sig::Vector _lowerBound;

            yarp::sig::Matrix _Aeq;
            yarp::sig::Vector _beq;

            yarp::sig::Matrix _Aineq;
            yarp::sig::Vector _bUpperBound;
            yarp::sig::Vector _bLowerBound;
        public:
            /**
             * @brief Aggregated
             * @param bounds a std::list of Bounds
             */
            Aggregated(const std::list< Bounds<yarp::sig::Matrix, yarp::sig::Vector, x_size> >& bounds);

            yarp::sig::Vector getLowerBound();
            yarp::sig::Vector getUpperBound();

            yarp::sig::Matrix getAeq();
            yarp::sig::Vector getbeq();

            yarp::sig::Matrix getAineq();
            yarp::sig::Vector getbLowerBound();
            yarp::sig::Vector getbUpperBound();

            void update(const yarp::sig::Vector &x);
        };
    }
 }
