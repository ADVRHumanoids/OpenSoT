/*
 * Copyright (C) 2014 Walkman
 * Author: Cheng Fang
 * email:  cheng.fang@iit.it
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

#ifndef SELFCOLLISIONAVOIDANCE_H
#define SELFCOLLISIONAVOIDANCE_H


 #include <OpenSoT/Constraint.h>
 #include <OpenSoT/tasks/velocity/Cartesian.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/collision_utils.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace velocity {

            /**
             * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
             *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
             *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
             *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1; 
             *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
             *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
             */
            class SelfCollisionAvoidance: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<SelfCollisionAvoidance> Ptr;
            protected: 
                /**
                 * @brief _LinkPair_threshold is the allowable minimum distance between every Link pair
                 */
                double _linkPair_threshold;
                /**
                 * @brief all the link pairs whose minimum distance are smaller than this "_Detection_threshold" would be dealt with further
                 */
                double _detection_threshold;
                iDynUtils& robot_col;
                ComputeLinksDistance computeLinksDistance;

                /**
                 * @brief base_index all the calculation and expression is described in
                 * a base link frame which is "waist" link frame
                 */
                int base_index;
            public:
                ///TODO: Move in cartesian utils
                // the following four functions are responsible for the transformation between the yarp data type and Eigen data type
                Eigen::MatrixXd from_yarp_to_Eigen_matrix(const yarp::sig::Matrix &Y_M);
                yarp::sig::Matrix from_Eigen_to_Yarp_matrix(const Eigen::MatrixXd &E_M);
                Eigen::VectorXd from_yarp_to_Eigen_vector(const yarp::sig::Vector &Y_V);
                yarp::sig::Vector from_Eigen_to_Yarp_vector(const Eigen::VectorXd &E_V);
                
                /**
                 * @brief Skew_symmetric_operator is used to get the transformation matrix which is used to transform
                 * the base Jacobian to goal Jacobian
                 * @param r_cp the vector measured from the origin of the reference link frame to the closest point
                 * @return Skew symmetric matrix of the input vector
                 * NOTE: the base Jacobian should be multiplied by the output matrix on its left side.
                 */
                Eigen::MatrixXd Skew_symmetric_operator (const Eigen::Vector3d & r_cp);

                /**
                 * @brief calculate_Aineq_bUpperB the core function which is used to update the variables Aineq and
                 * bUpperBound for this constraint
                 * @param x the robot current configuration vector
                 * @param Aineq_fc Aineq matrix of this constraint
                 * @param bUpperB_fc bUpperBound of this constraint
                 */
                void calculate_Aineq_bUpperB (const yarp::sig::Vector & x, yarp::sig::Matrix & Aineq_fc, yarp::sig::Vector & bUpperB_fc );

            public:
                /**
                 * @brief SelfCollisionAvoidanceConstraint
                 * @param x the robot current configuration vector
                 * @param robot the robot model reference
                 * @param Detection_threshold all the link pairs whose minimum distance are smaller than this Detection_threshold
                 * would be dealt with further
                 * @param linkPair_threshold the minimum distance between each Link pair
                 */
                SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       double detection_threshold = std::numeric_limits<double>::infinity(),
                                       double linkPair_threshold = 0.0);
                /**
                 * @brief getLinkPairThreshold
                 * @return _LinkPair_threshold
                 */
                double getLinkPairThreshold();

                /**
                 * @brief getDetectionThreshold
                 * @return _Detection_threshold
                 */
                double getDetectionThreshold();

                /**
                 * @brief setLinkPairThreshold set _LinkPair_threshold
                 * @param linkPair_threshold (always positive)
                 */
                void setLinkPairThreshold(const double linkPair_threshold);

                /**
                 * @brief setDetectionThreshold set _Detection_threshold
                 * @param detection_threshold (always positive)
                 */
                void setDetectionThreshold(const double detection_threshold);

                void update(const yarp::sig::Vector &x);


                /**
                 * @brief setCollisionWhiteList resets the allowed collision matrix by setting all collision pairs as disabled.
                 *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
                 *        which are disabled in the SRDF
                 * @param whiteList a list of links pairs for which to not check collision detection
                 * @return true on success
                 */
                bool setCollisionWhiteList(std::list< LinkPairDistance::LinksPair > whiteList);

                /**
                 * @brief setCollisionBlackList resets the allowed collision matrix by setting all collision pairs
                 *        (for which collision geometries are avaiable) as enabled.
                 *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
                 *        which are disabled in the SRDF
                 * @param blackList a list of links pairs for which to not check collision detection
                 * @return true on success
                 */
                bool setCollisionBlackList(std::list< LinkPairDistance::LinksPair > blackList);
            };
        }
    }
 }


#endif // SELFCOLLISIONAVOIDANCE_H

