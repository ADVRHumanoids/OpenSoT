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

#ifndef __BOUNDS_VELOCITY_CONVEXHULL_H__
#define __BOUNDS_VELOCITY_CONVEXHULL_H__

 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/CoM.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/convex_hull.h>
 #include <kdl/frames.hpp>

#define BOUND_SCALING 0.01

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The ConvexHull class implements a constraint of the type
             * \f$A_{\text{CH}}J_{\text{CoM}}\dot{q} \leq b_{\text{CH}}\f$, where every row in
             * \f$\left[ A_{\text{CH}} , -b_{\text{CH}\right]\f$
            */
            class ConvexHull: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<ConvexHull> Ptr;
            private:
                iDynUtils &_robot;
                double _boundScaling;
                idynutils::convex_hull _convex_hull;
                std::vector<KDL::Vector> _ch;
            public:
                /**
                 * @brief ConvexHull constructor
                 * @param x the robot configuration vector
                 * @param robot the robot model, with floating base link set on the support foot
                 * @param boundScaling the margin, in percentage, of the bounds margins
                 */
                ConvexHull( const yarp::sig::Vector& x,
                            iDynUtils& robot,
                            const double boundScaling = BOUND_SCALING);

                void update(const yarp::sig::Vector &x);

                /**
                 * @brief getConstraints returns A and b such that \f$A*\delta q < b\f$ implies staying in the convex hull
                 * @param points a list of points representing the convex hull
                 * @param A the matrix of constraints
                 * @param b the vector of coefficients
                 */
                static void getConstraints(const std::vector<KDL::Vector> &points,
                                            yarp::sig::Matrix& A, yarp::sig::Vector& b,
                                            const double boundScaling = BOUND_SCALING);

                /**
                 * @brief getLineCoefficients returns implicit form of the a line passing between p0 and p1
                 * @param p0 point 0
                 * @param p1 point 1
                 * @param a cofficient in the implicit representation of the rect as ax + by + c = 0
                 * @param b cofficient in the implicit representation of the rect as ax + by + c = 0
                 * @param c cofficient in the implicit representation of the rect as ax + by + c = 0
                 */
                static void getLineCoefficients(const KDL::Vector &p0, const KDL::Vector &p1,
                                                double &a, double& b, double &c);

                bool getConvexHull(std::vector<KDL::Vector>& ch);
            };
        }
    }
 }

#endif
