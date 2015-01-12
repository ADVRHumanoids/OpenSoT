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

#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

ConvexHull::ConvexHull(const yarp::sig::Vector& x,
                       iDynUtils& robot,
                       const double boundScaling) :
    Constraint(x.size()), _robot(robot),
    _boundScaling(boundScaling),
    _convex_hull() {

    this->update(x);
}

void ConvexHull::update(const yarp::sig::Vector &x) {

    /************************ COMPUTING BOUNDS ****************************/

    yarp::sig::Matrix JCoM;
    _robot.iDyn3_model.getCOMJacobian(JCoM);
    JCoM.removeCols(0,6);
    JCoM.removeRows(2,4);

    if(getConvexHull(_ch))
        this->getConstraints(_ch, _Aineq, _bUpperBound, _boundScaling);
    else
    {
        _Aineq.resize(0,_Aineq.cols());
        _bUpperBound.resize(0);}

    assert(JCoM.rows() == _Aineq.cols());

    _Aineq = _Aineq * JCoM;
    /**********************************************************************/
}

bool ConvexHull::getConvexHull(std::vector<KDL::Vector> &ch)
{   
    std::list<KDL::Vector> points;
    if(_robot.getSupportPolygonPoints(points)){
        std::vector<KDL::Vector> tmp_ch;
        if(_convex_hull.getConvexHull(points, tmp_ch)){
            ch = tmp_ch;
            return true;}
        else
            std::cout<<"Problems computing Convex Hull, old Convex Hull will be used"<<std::endl;
    }
    else
        std::cout<<"Problems getting Points for Convex Hull computation!, old Convex Hull will be used"<<std::endl;
    return false;
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

        //Where is the line w.r.t. the robot?
        //We consider that the constraint is feasable at the beginning (the robot is in the convex hull)
        if(_c <= 0.0) { // c < 0 --> AJdq < -c w/ -c > 0
            A(z,0) = + _a;
            A(z,1) = + _b;
            b[z] =   - _c;
        } else { // c > 0 --> -AJdq < c
            A(z,0) = - _a;
            A(z,1) = - _b;
            b[z] =   + _c;
        }


        double normalizedBoundScaling = boundScaling * sqrt(_a*_a + _b*_b); //boundScaling Normalization
        if(fabs(_c) <= normalizedBoundScaling)
            b[z] = 0.0;
        else
            b[z] -= normalizedBoundScaling;
        z++;
    }
//    std::cout<<"A_ch: "<<A.toString()<<std::endl;
//    std::cout<<"b_ch: "<<b.toString()<<std::endl;
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


