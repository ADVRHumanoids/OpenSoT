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
 #include <drc_shared/idynutils.h>
 #include <drc_shared/utils/convex_hull.h>
 #include <kdl/frames.hpp>

#define BOUND_SCALING 0.01

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            class ConvexHull: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            private:
                boost::shared_ptr<OpenSoT::tasks::velocity::CoM> _com;
                iDynUtils& _robot;
                double _boundScaling;
                drc_shared::convex_hull _convex_hull;
                int _support_foot_linkIndex;
            public:
                /**
                 * @brief ConvexHull constructor
                 * @param robot the robot model
                 */
                ConvexHull( iDynUtils& robot,
                            const unsigned int x_size,
                            const double boundScaling = BOUND_SCALING);

                /**
                 * @brief ConvexHull constructor
                 * @param com the CoM task that provides us with CoM jacobian
                 */
                ConvexHull(boost::shared_ptr<OpenSoT::tasks::velocity::CoM> com,
                           const unsigned int x_size,
                           const double boundScaling = BOUND_SCALING);

                void update();

                /**
                 * @brief getConstraints returns A and b such that A*\delta q < b implies staying in the convex hull
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

                void getConvexHull(std::vector<KDL::Vector>& ch);
            };
        }
    }
 }

#endif
