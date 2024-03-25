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
#include <OpenSoT/utils/convex_hull_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::constraints::velocity;

ConvexHull::ConvexHull(XBot::ModelInterface& robot,
                       const std::list<std::string>& links_in_contact,
                       const double safetyMargin) :
    Constraint("convex_hull", robot.getNv()),
    _links_in_contact(links_in_contact),_robot(robot),
    _boundScaling(safetyMargin),
    _JCoM(3, _x_size),
    _C(links_in_contact.size(), 2)
{
    _convex_hull = std::make_shared<convex_hull>();
    _bUpperBound.resize(links_in_contact.size());
    _bLowerBound.resize(links_in_contact.size());
    _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bUpperBound.size());
    this->update();
}

void ConvexHull::update() {

    /************************ COMPUTING BOUNDS ****************************/

    _robot.getCOMJacobian(_JCoM);

    if(getConvexHull(_ch))
        this->getConstraints(_ch, _C, _bUpperBound, _boundScaling);



    _Aineq = _C * _JCoM.block(0,0,2,_x_size);
    //_bLowerBound = -1.0e20*_bLowerBound.setOnes(_bUpperBound.size());
    /**********************************************************************/
}

bool ConvexHull::getConvexHull(std::vector<Eigen::Vector3d> &ch)
{
    _points.clear();
    // get support polygon points w.r.t. COM
    if(_convex_hull->getSupportPolygonPoints(_points,_links_in_contact,_robot,"COM")){

        if(_points.size() > 2)
        {
            _tmp_ch.clear();
            if(_convex_hull->getConvexHull(_points, _tmp_ch)){
                ch = _tmp_ch;
                return true;}
            else
                XBot::Logger::info("Problems computing Convex Hull, old Convex Hull will be used\n");
        }
        else
            XBot::Logger::info("Too few points for Convex Hull computation!, old Convex Hull will be used");
    }
    else
        XBot::Logger::info("Problems getting Points for Convex Hull computation!, old Convex Hull will be used");

    return false;
}


void ConvexHull::getConstraints(const std::vector<Eigen::Vector3d> &convex_hull,
                                Eigen::MatrixXd &A, Eigen::VectorXd &b,
                                const double boundScaling)
{
    double _a, _b, _c;
    A.setZero(A.rows(), A.cols());
    b = 1.0e10*b.setOnes(b.size());
//    A.resize(convex_hull.size(),2);
//    b.resize(convex_hull.size());

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
            b(z) =   - _c;
        } else { // c > 0 --> -AJdq < c
            A(z,0) = - _a;
            A(z,1) = - _b;
            b(z) =   + _c;
        }


        double normalizedBoundScaling = boundScaling * sqrt(_a*_a + _b*_b); //boundScaling Normalization
//        if(fabs(_c) <= normalizedBoundScaling)
//            b(z) = 0.0;
//        else
            b(z) -= normalizedBoundScaling;
        z++;
    }
    //std::cout<<"A_ch: "<<A<<std::endl;
    //std::cout<<"b_ch: "<<b<<std::endl;
}

void ConvexHull::getLineCoefficients(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
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


void ConvexHull::setSafetyMargin(const double safetyMargin)
{
    _boundScaling = safetyMargin;
}
