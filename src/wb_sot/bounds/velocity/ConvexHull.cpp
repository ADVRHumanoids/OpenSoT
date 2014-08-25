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

#include <wb_sot/bounds/velocity/ConvexHull.h>
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace wb_sot::bounds::velocity;
using namespace yarp::math;

ConvexHull::ConvexHull(iDynUtils &robot,
                       const unsigned int x_size,
                       const double boundScaling) :
    Bounds(x_size), _robot(robot),
    _boundScaling(boundScaling),
    _convex_hull() {

    this->update();
}

void ConvexHull::update() {

    /************************ COMPUTING BOUNDS ****************************/

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    drc_shared::convex_hull::getSupportPolygonPoints(_robot, points);
    _convex_hull.getConvexHull(points, ch);
    this->getConstraints(ch, _Aineq, _bUpperBound, _boundScaling);

    /**********************************************************************/
}


void ConvexHull::getConstraints(const std::vector<KDL::Vector> &convex_hull,
                                yarp::sig::Matrix &A, yarp::sig::Vector &b,
                                const double boundScaling)
{
    double _a, _b, _c;
    A.resize(convex_hull.size(),2);
    b.resize(convex_hull.size());

    unsigned int z = 0;

    for(unsigned int j = 0; j < convex_hull.size(); ++j)
    {
        unsigned int k = (j + 1)%convex_hull.size();
        getLineCoefficients(convex_hull[j], convex_hull[k], _a, _b, _c);
        if(_c <= 0.0) { // see Moleskine
            A(z,0) = + _a;
            A(z,1) = + _b;
            b[z] =   - _c;
        } else {
            A(z,0) = - _a;
            A(z,1) = - _b;
            b[z] =   + _c;
        }
        if(fabs(_c) <= boundScaling)
            b[z] = 0.0;
        else
            b[z] -= boundScaling;
        z++;
    }
}

void ConvexHull::getLineCoefficients(const KDL::Vector &p0, const KDL::Vector &p1,
                                     double &a, double &b, double&c)
{
    double x1 = p0.x();
    double x2 = p1.x();
    double y1 = p0.y();
    double y2 = p1.y();

    a = y1 - y2;
    b = x2 - x1;
    c = -b*y1 -a*x1;
}


