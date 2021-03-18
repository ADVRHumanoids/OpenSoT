/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi, Cheng Fang, Arturo Laurenzi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it, arturo.laurenzi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef OPENSOT_COLLISION_UTILS_H_
#define OPENSOT_COLLISION_UTILS_H_

#include <kdl/frames.hpp>
#include <limits>
#include <list>
#include <string>
#include <utility>
#include <XBotInterface/ModelInterface.h>
#include <srdfdom_advr/model.h>
#include <fcl/config.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/make_shared.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/PlanningSceneWorld.h>

// construct vector
inline KDL::Vector toKdl(urdf::Vector3 v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
inline KDL::Rotation toKdl(urdf::Rotation r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
inline KDL::Frame toKdl(urdf::Pose p)
{
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

/**
 * @brief The LinkPairDistance class represents the minimum distance information
 * between two links.
 * The two links must have shape (i.e. collision) information.
 * Instances of this class will contain the name of both links in the pair,
 * the actual minimum distance between the two, and the homogeneous transforms
 * in the respective link frame reference system of the minimum distance point
 * in each shape.
 */
class LinkPairDistance
{

public:

    typedef std::pair<std::string, std::string> LinksPair;

    /**
     * @brief LinkPairDistance creates an instance of a link pair distance data structure
     * @param link1 the first link name
     * @param link2 the second link name
     * @param w_T_closestPoint1 the transform from the world frame
     * to the closest point on its shape
     * @param w_T_closesPoint2 the transform from the world frame
     * to the closest point on its shape
     * @param distance the distance between the two minimum distance points
     */
    LinkPairDistance(const std::string& link1,
                     const std::string& link2,
                     const KDL::Frame& w_T_closestPoint1,
                     const KDL::Frame& w_T_closestPoint2,
                     const double& distance);

    /**
     * @brief isLink2WorldObject returns true if link2 refers
     * to an environment object, and not a robot link. On the other hand,
     * link1 is always a robot link
     */
    bool isLink2WorldObject() const;

    /**
     * @brief getDistance returns the minimum distance between the two link shapes
     * @return a double representing the minimum distance
     */
    const double& getDistance() const;

    /**
     * @brief getLink_T_closestPoint returns a pair of homogeneous transformation
     * from a link reference frame to the closest point on the respective link shape
     * @return a pair of homogeneous transformations
     */
    const std::pair<KDL::Frame, KDL::Frame>& getClosestPoints() const;

    /**
     * @brief getLinkNames returns the pair of links between which we want
     * express the distance information
     * @return a pair of strings
     */
    const std::pair<std::string, std::string>& getLinkNames() const;

    /**
     * @brief operator< is the comparison operator between two LinkDIstancePairs.
     * It returns true if the first link pair is closer than the second pair.
     * If the distance is exactly the same, the pairs will be sorted by
     * alphabetic order using the first link name in each pair
     * @param second the second link pair
     * @return true if first pair is closer than second pair
     */
    bool operator <(const LinkPairDistance& second) const;

private:
    /**
     * @brief _link_pair the pair of link names in alphabetic order
     */
    LinksPair _link_pair;

    /**
     * @brief link_T_closestPoint is a pair of homogeneous transformations.
     *        The first transformation will express the pose of the closes point on the first link shape
     *        in the link reference frame, i.e. link1_T_closestPoint1.
     *        The second transformation will express the pose of the closes point on the second link shape
     *        in the link reference frame, i.e. link2_T_closestPoint2.
     */
    std::pair<KDL::Frame, KDL::Frame> _closest_points;

    /**
     * @brief distance the minimum distance between the two link shapes, i.e.
     * ||w_T_closestPoint1.p - w_T_closesPoint2.p||
     */
    double distance;

};

class ComputeLinksDistance
{

public:

    /**
     * @brief ComputeLinksDistance is an utility class to retrieve distances between
     * robot link collisions, as well as between robot links and the environment
     * @param model to retrieve kinematic information (link poses)
     * @param collision_urdf if passed, overrides the collision information of model
     * @param collision_srdf if passed, overrides the model's allowed collisions
     */
    ComputeLinksDistance(const XBot::ModelInterface& model,
                         urdf::ModelConstSharedPtr collision_urdf = nullptr,
                         srdf::ModelConstSharedPtr collision_srdf = nullptr);

    /**
     * @brief getLinkDistances returns a list of distances between all link pairs
     * which are enabled for checking.
     * If detectionThreshold is not infinity, the list will be clamped to contain only
     * the pairs whose distance is smaller than the detection threshold.
     * @param detectionThreshold the maximum distance which we use to look for
     * link pairs.
     * @return a sorted list of linkPairDistances
     */
    std::list<LinkPairDistance> getLinkDistances(double detectionThreshold = std::numeric_limits<double>::infinity());

    /**
     * @brief setCollisionWhiteList resets the allowed collision matrix by setting all collision pairs as disabled.
     *        It then enables all collision pairs specified in the whiteList. Lastly it will disable all collision pairs
     *        which are disabled in the SRDF
     * @param whiteList a list of links pairs for which to not check collision detection
     * @return
     */
    bool setCollisionWhiteList(std::list< LinkPairDistance::LinksPair > whiteList);

    /**
     * @brief setCollisionBlackList resets the allowed collision matrix by setting all collision pairs
     *        (for which collision geometries are avaiable) as enabled.
     *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
     *        which are disabled in the SRDF
     * @param blackList a list of links pairs for which to not check collision detection
     * @return
     */
    bool setCollisionBlackList(std::list< LinkPairDistance::LinksPair > blackList);

    /**
     * @brief setWorldCollisions
     * @param wc
     * @return
     */
    bool setWorldCollisions(const moveit_msgs::PlanningSceneWorld& wc);

    /**
     * @brief KDL2fcl ceonverts a kdl transform into a fcl transform
     * @param in a KDL::Frame
     * @return  fcl::Transform3f
     */
    static fcl::Transform3<double> KDL2fcl(const KDL::Frame &in);

    /**
     * @brief fcl2KDL converts a fcl transform into a kdl transform
     * @param in a fcl::Transform3f
     * @return a KDL::Frame
     */
    static KDL::Frame fcl2KDL(const fcl::Transform3d& in);

    friend class TestCapsuleLinksDistance;

    /**
     * @brief The Capsule class represents a capsule shape expressed in an
     *  arbitrary frame
     */
    class Capsule
    {
        KDL::Vector ep1;
        KDL::Vector ep2;
        double radius;
        double length;

        /**
         * @brief Capsule constructs a capsule object
         * @param origin the frame located in the end-point number 1, with z-axis aligned with the capsule axis
         * @param radius the capsule radius
         * @param length the capsule length
         */
        Capsule(const KDL::Frame& origin, const double radius, const double length) :
            radius(radius), length(length)
        {
            ep1 = origin.p;
            ep2 = origin.p + length * origin.M.UnitZ();
        }

        double getLength() { return this->length; }

        double getRadius() { return this->radius; }

        void getEndPoints(KDL::Vector& ep1, KDL::Vector& ep2)
        {
            ep1 = this->ep1; ep2 = this->ep2;
        }
    };

    class LinksPair
    {

    public:

        std::string linkA;
        std::string linkB;
        boost::shared_ptr<fcl::CollisionObjectd> collisionObjectA;
        boost::shared_ptr<fcl::CollisionObjectd> collisionObjectB;

        LinksPair(ComputeLinksDistance* const father,
                  std::string linkA,
                  std::string linkB):
            linkA(linkA), linkB(linkB)
        {
            collisionObjectA = father->_collision_obj.at(linkA);
            collisionObjectB = father->_collision_obj.at(linkB);
        }

    };

    friend class ComputeLinksDistance::LinksPair;

private:

    /**
     * @brief _acm
     */
    collision_detection::AllowedCollisionMatrixPtr _acm;

    /**
     * @brief model a reference to the robot model. We expect it to be updated
     * before calling getLinkDistances
     */
    const XBot::ModelInterface& _model;

    /**
     * @brief moveit_robot_model
     */
    robot_model::RobotModelConstPtr _moveit_model;

    /**
     * @brief robot_srdf is used to load the ACM every time a whiteList or blackList is generated
     */
    srdf::ModelSharedPtr _srdf;

    /**
     * @brief robot urdf
     */
    urdf::ModelSharedPtr _urdf;

    /**
     * @brief _collision_obj a map of collision objects
     */
    std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd>> _collision_obj;

    /**
     * @brief _env_obj_names is the set of the names of all objects
     * that make up the environment collision model; the corresponding
     * fcl collision is found inside the collision_objects_ member
     */
    std::set<std::string> _env_obj_names;

    /**
     * @brief link_T_shape a map of transforms from link frame to shape frame.
     * Notice how the shape frame is always the center of the shape,
     * except for the capsule, where it lies on one endpoint
     */
    std::map<std::string,KDL::Frame> _link_T_shape;

    /**
     * @brief globalToLinkCoordinates transforms a fcl::Transform3f frame to a KDL::Frame in the link reference frame
     * @param linkName the link name representing a link reference frame
     * @param w_T_f fcl::Transform3f representing a frame in a global reference frame
     * @param link_T_f a KDL::Frame representing a frame in link reference frame
     * @return true on success
     */
    bool globalToLinkCoordinates(const std::string& linkName,
                                 const fcl::Transform3<double>& w_T_f,
                                 KDL::Frame& link_T_f);

    /**
     * @brief shapeToLinkCoordinates transforms a fcl::Transform3f frame to a KDL::Frame in the link reference frame
     * @param linkName the link name representing a link reference frame
     * @param w_T_f fcl::Transform3f representing a frame in the shape reference frame
     * @param link_T_f a KDL::Frame representing a frame in link reference frame
     * @return true on success
     */
    bool shapeToLinkCoordinates(const std::string &linkName,
                                const fcl::Transform3<double> &fcl_shape_T_f,
                                KDL::Frame &link_T_f);


    /**
     * @brief parseCollisionObjects
     * @return true on success
     */
    bool parseCollisionObjects();

    /**
     * @brief updateCollisionObjects updates all collision objects with correct transforms (link_T_shape)
     * @return true on success
     */
    bool updateCollisionObjects();

    /**
     * @brief generateLinksToUpdate generates a list of links for which we query w_T_link
     */
    void generateLinksToUpdate();

    /**
     * @brief linksToUpdate a list of links to update
     */
    std::set<std::string> _links_to_update;

    /**
     * @brief generatePairsToCheck generates a list of pairs to check for distance
     */
    void generatePairsToCheck();

    /**
     * @brief pairsToCheck a list of pairs to check for collision detection
     */
    std::list<ComputeLinksDistance::LinksPair> _pairs_to_check;

    /**
     * @brief loadDisabledCollisionsFromSRDF disabled collisions between links as specified in the robot srdf.
     *        Notice this function will not reset the acm, rather just disable collisions that are flagged as
     *        "disabled" in the robot srdf
     * @param srdf the srdf file to use to load the ACM
     * @param acm the allowed collision matrix to modify according to the srdf info
     * @TODO we should move this in collision_utils together with checkSelfCollision and checkSelfCollisionAt
     */
    void loadDisabledCollisionsFromSRDF(const srdf::Model& srdf,
                                        collision_detection::AllowedCollisionMatrixPtr acm);


};

#endif
