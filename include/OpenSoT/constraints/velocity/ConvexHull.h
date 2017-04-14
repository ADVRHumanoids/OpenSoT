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
 #include <kdl/frames.hpp>
 #include <Eigen/Dense>
 #include <XBotInterface/ModelInterface.h>
 #include <OpenSoT/utils/convex_hull_utils.h>

#define BOUND_SCALING 0.01

 // idynutils::convex_hull forward declaration
 namespace idynutils {
     class convex_hull;
 }

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {
            /**
             * @brief The ConvexHull class implements a constraint of the type
             * \f$A_{\text{CH}}J_{\text{CoM}}\dot{q} \leq b_{\text{CH}}\f$, where every row in
             * \f$\left[ A_{\text{CH}} , -b_{\text{CH}}\right]\f$
            */
            class ConvexHull: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<ConvexHull> Ptr;
            private:
                XBot::ModelInterface &_robot;
                double _boundScaling;
                boost::shared_ptr<convex_hull> _convex_hull;
                std::vector<KDL::Vector> _ch;
                std::list<std::string> _links_in_contact;

            public:
                /**
                 * @brief ConvexHull constructor
                 * @param x the robot configuration vector
                 * @param robot the robot model, with floating base link set on the support foot
                 * @param safetyMargin the margin, in [m], of the bounds margins
                 */
                ConvexHull( const Eigen::VectorXd& x,
                            XBot::ModelInterface& robot,
                            const std::list<std::string>& links_in_contact,
                            const double safetyMargin = BOUND_SCALING);

                /**
                 * @brief getConstraints returns A and b such that \f$A*\delta q < b\f$ implies staying in the convex hull
                 * @param points a list of points representing the convex hull
                 * @param A the matrix of constraints
                 * @param b the vector of coefficients
                 */
                static void getConstraints(const std::vector<KDL::Vector> &points,
                                            Eigen::MatrixXd& A, Eigen::VectorXd& b,
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

                /**
                 * @brief setSafetyMargin sets a safety margin in [m] for the convex hull
                 * @param sagetyMargin
                 */
                void setSafetyMargin(const double safetyMargin);

                void update(const Eigen::VectorXd &x);

                std::list<std::string> getLinksInContact()
                {
                    return _links_in_contact;
                }

                void setLinksInContact(const std::list<std::string>& links_inc_contact)
                {
                    _links_in_contact = links_inc_contact;
                }
            };
        }
    }
 }

#endif
