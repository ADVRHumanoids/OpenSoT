/*
 * Copyright (C) 2014 Walkman
 * Author: Yangwei YOU
 * email:  yangwei.you@foxmail.com
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

#ifndef COLLISIONAVOIDANCE_H
#define COLLISIONAVOIDANCE_H

#include <OpenSoT/Constraint.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <kdl/frames.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Dense>

 namespace OpenSoT {
    namespace constraints {
        namespace velocity {

            /**
             * @brief The CollisionAvoidance class implements a constraint of Collision Avoidance especially for external environment objects
             *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
             *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
             *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1; 
             *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
             *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
             */
            class CollisionAvoidance: public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {
            public:
                typedef boost::shared_ptr<CollisionAvoidance> Ptr;
            protected:
                double _boundScaling;
                /**
                 * @brief _LinkPair_threshold is the allowable minimum distance between every Link pair
                 */
                double _linkPair_threshold;
                /**
                 * @brief all the link pairs whose minimum distance are smaller than this "_Detection_threshold" would be dealt with further
                 */
                double _detection_threshold;
                XBot::ModelInterface& robot_col;
//                 ComputeLinksDistance computeLinksDistance;
                
                /**
                 * @brief link pairs for collision check, [link belonging to robot, link in the envionment]
                 * 
                 */
                std::list< LinkPairDistance::LinksPair > _interested_link_pairs;
                std::vector< std::string > _interested_links;
//                 std::vector< std::string > _environment_link;

                /**
                 * @brief linksToUpdate a list of links to update
                 */
                std::set<std::string> linksToUpdate;
    
                /**
                 * @brief collision_objects_ a map of collision objects
                 */
                std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd> > collision_objects_;
                
                /**
                 * @brief shapes_ is a map of collision geometries
                 */
//                 std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd> > shapes_;
    
                /**
                 * @brief link_T_shape a map of transforms from link frame to shape frame.
                 *        Notice how the shape frame is always the center of the shape,
                 *        except for the capsule, where it lies on one endpoint
                 */
                std::map<std::string,KDL::Frame> link_T_shape;
    
                /**
                 * @brief _x_cache is a copy of last q vector used to update the constraints.
                 * It is used in order to avoid multiple updated when the constraint is present
                 * in multiple tasks in the same stack. It is a simple speedup waiting for the stack system to support
                 * constraint caching - or for idynutils to support some form of cache system
                 */
                Eigen::VectorXd _x_cache;

                /**
                 * @brief base_name all the calculation and expression is described in
                 * a base link frame
                 */
                std::string base_name;

                Eigen::MatrixXd _J_transform;
            public:               
                /**
                 * @brief Skew_symmetric_operator is used to get the transformation matrix which is used to transform
                 * the base Jacobian to goal Jacobian
                 * @param r_cp the vector measured from the origin of the reference link frame to the closest point
                 * @param J_transform Skew symmetric matrix of the input vector
                 * NOTE: the base Jacobian should be multiplied by the output matrix on its left side.
                 */
                void skewSymmetricOperator (const Eigen::Vector3d & r_cp, Eigen::MatrixXd& J_transform);

                /**
                 * @brief calculate_Aineq_bUpperB the core function which is used to update the variables Aineq and
                 * bUpperBound for this constraint
                 * @param Aineq_fc Aineq matrix of this constraint
                 * @param bUpperB_fc bUpperBound of this constraint
                 */
                void calculate_Aineq_bUpperB (Eigen::MatrixXd & Aineq_fc, Eigen::VectorXd & bUpperB_fc );

            public:
                /**
                 * @brief CollisionAvoidanceConstraint
                 * @param x the robot current configuration vector
                 * @param robot the robot model reference
                 * @param base_link all the calculation and expression is described in
                 * this frame
                 * @param Detection_threshold all the link pairs whose minimum distance are smaller than this Detection_threshold
                 * would be dealt with further
                 * @param linkPair_threshold the minimum distance between each Link pair
                 * @param boundScaling the bound scaling for the capsule distance (a lower number means we will approach
                 *        the linkPair_threshold more slowly)
                 */
                CollisionAvoidance(const Eigen::VectorXd& x,
                                       XBot::ModelInterface &robot,
                                       std::string& base_link,
                                       const std::vector<std::string> &interested_robot_links,
				       const std::map<std::string, Eigen::Affine3d> &envionment_collision_frames,
//                                        const std::map<std::string, boost::shared_ptr<fcl::CollisionObject>> &envionment_collision_objects,
                                       const double &detection_threshold = std::numeric_limits<double>::infinity(),
                                       const double &linkPair_threshold = 0.0,
                                       const double &boundScaling = 1.0);
                /**
                * @brief parseCollisionObjects
                * @param robot_urdf_path a string representing the robot urdf with collision information
                * @param robot_srdf_path a string representing the robot srdf with collision information (i.e. ACM)
                * @return true on success
                */
                bool parseCollisionObjects();

                /**
                 * @brief updateCollisionObjects updates all collision objects with correct transforms (link_T_shape)
                 * @return true on success
                 */
                 bool updateCollisionObjects();

                /**
                 * @brief setEnvironmentCollisionObjects updates all environment collision objects with correct transforms (link_T_shape)
                 * @return true on success
                 */
                 bool updateEnvironmentCollisionObjects(const std::map<std::string, KDL::Frame> &envionment_collision_frames);
		 
                /**
                * @brief getLinkDistances returns a list of distances between all link pairs which are enabled for checking.
                *                         If detectionThreshold is not infinity, the list will be clamped to contain only
                *                         the pairs whose distance is smaller than the detection threshold.
                * @return a sorted list of linkPairDistances
                */
                std::list<LinkPairDistance> getLinkDistances(const double &detectionThreshold);

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
                void setLinkPairThreshold(const double &linkPair_threshold);

                /**
                 * @brief setDetectionThreshold set _Detection_threshold
                 * @param detection_threshold (always positive)
                 */
                void setDetectionThreshold(const double &detection_threshold);

                /**
                 * @brief globalToLinkCoordinates transforms a fcl::Transform3d frame to a KDL::Frame in the link reference frame
                 * @param linkName the link name representing a link reference frame
                 * @param w_T_f fcl::Transform3d representing a frame in a global reference frame
                 * @param link_T_f a KDL::Frame representing a frame in link reference frame
                 * @return true on success
                 */
                bool globalToLinkCoordinates(const std::string& linkName,
                                             const fcl::Transform3d& w_T_f,
                                             KDL::Frame& link_T_f);

                /**
                 * @brief shapeToLinkCoordinates transforms a fcl::Transform3d frame to a KDL::Frame in the link reference frame
                 * @param linkName the link name representing a link reference frame
                 * @param w_T_f fcl::Transform3d representing a frame in the shape reference frame
                 * @param link_T_f a KDL::Frame representing a frame in link reference frame
                 * @return true on success
                 */
                bool shapeToLinkCoordinates(const std::string &linkName,
                                const fcl::Transform3d &fcl_shape_T_f,
                                KDL::Frame &link_T_f);
    
                /**
                 * @brief update recomputes Aineq and bUpperBound if x is different than the previously stored value
                 * @param x the state vector. It gets cached so that we won't recompute capsules distances if x didn't change
                 * TODO if the capsules distances are given by a server (i.e. a separate thread updaing capsules at a given rate)
                 * then the inelegant caching of x is not needed anymore (notice that, however, it would be nice to have a caching system
                 * at the stack level so that, if two tasks in the same stack are constrained by the same constraint, this constraint
                 * gets updated only once per stack update)
                 */
                void update(const Eigen::VectorXd &x);

                /**
                 * @brief setBoundScaling sets bound scaling for the capsule constraint
                 * @param boundScaling is a number which should be lower than 1.0
                 *        (e.g. 1./2. means we are looking two steps ahead and will avoid
                 *         collision with the capsule by slowing down)
                 */
                void setBoundScaling(const double &boundScaling);
            };
        }
    }
 }


#endif