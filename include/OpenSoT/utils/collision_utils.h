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
#include <memory>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>

#include <OpenSoT/utils/LinkPairDistance.h>

#include <map>

using namespace moveit::core;

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
     * @brief add/remove world collision objects according to the given planning
     * scene world
     * @return true if all requests (additions, deletions) could be performs
     * succesfully, false on (partial) insuccess
     */
    bool setWorldCollisions(const moveit_msgs::msg::PlanningSceneWorld& wc);

    /**
     * @brief add single collision to the world
     * @param id is the unique collision id
     * @param fcl_obj is the fcl collision object (geometry + transform)
     * @return true if input is valid
     */
    bool addWorldCollision(const std::string& id,
                           std::shared_ptr<fcl::CollisionObjectd> fcl_obj);

    /**
     * @brief remove world collision with given id
     */
    bool removeWorldCollision(const std::string& id);

    /**
     * @brief remove all world collisions
     */
    void removeAllWorldCollision();

    /**
     * @brief change the transform w.r.t. the world for the given
     * world collision
     */
    bool moveWorldCollision(const std::string& id,
                            KDL::Frame new_pose);

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

    class LinksPair
    {

    public:

        std::string linkA;
        std::string linkB;
        std::shared_ptr<fcl::CollisionObjectd> collisionObjectA;
        std::shared_ptr<fcl::CollisionObjectd> collisionObjectB;

        LinksPair(ComputeLinksDistance* const father,
                  std::string linkA,
                  std::string linkB);

    };

    friend class ComputeLinksDistance::LinksPair;

    std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> getCollisionObjects(){ return _collision_obj;}

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
     * @brief updateCollisionObjects updates all collision objects with correct transforms (link_T_shape)
     * @return true on success
     */
    bool updateCollisionObjects();

    std::map<std::string,KDL::Frame> getLinkToShapeTransforms();

    void setLinksVsEnvironment(const std::list<std::string>& links);

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
    RobotModelConstPtr _moveit_model;

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
    std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> _collision_obj;

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
     * @brief generateLinksToUpdate generates a list of links for which we query w_T_link
     */
    void generateLinksToUpdate();

    /**
     * @brief _links_to_update contains all robot links that can participate
     * in a collision (either with the environment or with other links)
     */
    std::set<std::string> _links_to_update;

    /**
     * @brief _links_vs_environment list of links to check against environment
     */
    std::set<std::string> _links_vs_environment;

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
     * Notice this function will not reset the acm, rather just disable collisions that are flagged as
     * "disabled" in the robot srdf
     * @param srdf the srdf file to use to load the ACM
     * @param acm the allowed collision matrix to modify according to the srdf info
     * @TODO we should move this in collision_utils together with checkSelfCollision and checkSelfCollisionAt
     */
    void loadDisabledCollisionsFromSRDF(const srdf::Model& srdf,
                                        collision_detection::AllowedCollisionMatrixPtr acm);


};

#endif
