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

#include <wb_sot/bounds/BilateralConstraint.h>

#include <yarp/math/Math.h>
#include <assert.h>
#include <limits>

using namespace wb_sot::bounds;
using namespace yarp::math;

BilateralConstraint::BilateralConstraint(const yarp::sig::Matrix &Aineq,
                                         const yarp::sig::Vector &bLowerBound,
                                         const yarp::sig::Vector &bUpperBound) :
    Bounds(Aineq.cols())
{
    _Aineq = Aineq;
    _bLowerBound = bLowerBound;
    _bUpperBound = bUpperBound;

    assert(_Aineq.rows() > 0);
    assert( (_Aineq.rows() == bLowerBound.size()) &&
            (_Aineq.rows() == bUpperBound.size()));
}


