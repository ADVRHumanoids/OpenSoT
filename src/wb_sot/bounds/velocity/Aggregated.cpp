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

#include <wb_sot/bounds/velocity/Aggregated.h>

#include <yarp/math/Math.h>
#include <assert.h>

using namespace wb_sot::bounds::velocity;

Aggregated::Aggregated(const std::list< BoundType* >& bounds,
                       const unsigned int x_size) :
    Bounds(x_size), _bounds(bounds)
{
    /* calling update to generate bounds */
    update(_x);
}

void Aggregated::update(const yarp::sig::Vector& x) {
    _x = x;

    /* resetting all internal data */
    _upperBound = yarp::sig::Vector(0);
    _lowerBound = yarp::sig::Vector(0);

    _Aeq = yarp::sig::Matrix(0,0);
    _beq = yarp::sig::Vector(0);

    _Aineq = yarp::sig::Matrix(0,0);
    _bUpperBound = yarp::sig::Vector(0);
    _bLowerBound = yarp::sig::Vector(0);

    /* iterating on all bounds.. */
    for(typename std::list< BoundType* >::iterator i = _bounds.begin();
        i != _bounds.end(); i++) {

        BoundType* b = *i;
        /* update bounds */
        b->update(_x);

        yarp::sig::Vector boundUpperBound = b->getUpperBound();
        yarp::sig::Vector boundLowerBound = b->getLowerBound();

        yarp::sig::Matrix boundAeq = b->getAeq();
        yarp::sig::Vector boundbeq = b->getbeq();

        yarp::sig::Matrix boundAineq = b->getAineq();
        yarp::sig::Vector boundbUpperBound = b->getbUpperBound();
        yarp::sig::Vector boundbLowerBound = b->getbLowerBound();

        /* copying lowerBound, upperBound */
        if(boundUpperBound.size() != 0 ||
           boundLowerBound.size() != 0) {
            assert(boundUpperBound.size() == _x_size);
            assert(boundLowerBound.size() == _x_size);

            if(_upperBound.size() == 0 ||
               _lowerBound.size() == 0) { // first valid bounds found
                assert(_upperBound.size() == _lowerBound.size());
                _upperBound = boundUpperBound;
                _lowerBound = boundLowerBound;
            } else {
                for(unsigned int i = 0; i < _x_size; ++i) {
                    // compute the minimum between current and new upper bounds
                    _upperBound[i] = std::min(  _upperBound[i],
                                                boundUpperBound[i]);
                    // compute the maximum between current and new lower bounds
                    _lowerBound[i] = std::max(  _lowerBound[i],
                                                boundLowerBound[i]);
                }
            }
        }

        /* copying Aeq, beq */
        if( boundAeq.rows() != 0 ||
            boundbeq.size() != 0) {
            assert(boundAeq.rows() == boundbeq.size());
            _Aeq = yarp::math::pile(_Aeq, boundAeq);
            _beq = yarp::math::cat(_beq, boundbeq);
        }

        /* copying Aineq, bUpperBound, bLowerBound*/
        if( boundAineq.rows() != 0 ||
            boundbUpperBound.size() != 0 ||
            boundbLowerBound.size() != 0) {

            assert(boundAineq.rows() == boundbUpperBound.size());
            assert(boundAineq.rows() == boundbLowerBound.size());

            _Aineq = yarp::math::pile(_Aineq, boundAineq);
            _bUpperBound = yarp::math::cat(_bUpperBound, boundbUpperBound);
            _bLowerBound = yarp::math::cat(_bUpperBound, boundbLowerBound);
        }
    }

    /* checking everything went fine */
    assert(_lowerBound.size() == 0 || _lowerBound.size() == _x_size);
    assert(_upperBound.size() == 0 || _upperBound.size() == _x_size);

    assert(_Aeq.rows() == _beq.size());
    assert(_Aeq.cols() == _x_size);

    assert(_Aineq.rows() == _bUpperBound.size());
    assert(_Aineq.rows() == _bLowerBound.size());
    assert(_Aineq.cols() == _x_size);
}

