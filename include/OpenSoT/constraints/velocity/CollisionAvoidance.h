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

#ifndef COLLISIONAVOIDANCE_H
#define COLLISIONAVOIDANCE_H

#include <OpenSoT/Constraint.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/collision.h>

#include <srdfdom/model.h>
#include <Eigen/Dense>

#include <moveit_msgs/PlanningSceneWorld.h>

namespace OpenSoT { namespace constraints { namespace velocity {

/**
 * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
 *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
 *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
 *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1;
 *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
 *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
 */
class CollisionAvoidance: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    typedef std::shared_ptr<CollisionAvoidance> Ptr;
    typedef std::pair<std::string, std::string> LinksPair;
    typedef XBot::Collision::CollisionModel::WitnessPointVector WitnessPointVector;
    typedef XBot::Collision::CollisionModel::LinkPairVector LinkPairVector;

    /**
     * @brief SelfCollisionAvoidance
     * @param x status
     * @param robot model
     * @param max_pairs maximum number of link pairs checked for SCA (-1 for all)
     * @param collision_urdf urdf used for collision model
     * @param collision_srdf srdf used for collision model
     * NOTE: if urdf and srdf are not specified, the one inside the robot model will be used
     */
    CollisionAvoidance(const XBot::ModelInterface& robot,
                       int max_pairs = -1,
                       urdf::ModelConstSharedPtr collision_urdf = nullptr,
                       srdf::ModelConstSharedPtr collision_srdf = nullptr);

    /**
     * @brief getLinkPairThreshold distance offset between two link pairs
     * @return _LinkPair_threshold distance offset between two link pairs
     */
    double getLinkPairThreshold();

    /**
     * @brief getDetectionThreshold
     * @return _Detection_threshold the maximum distance which we use to look for link pairs
     */
    double getDetectionThreshold();

    /**
     * @brief setLinkPairThreshold set _LinkPair_threshold distance offset between two link pairs
     * @param linkPair_threshold (always positive) distance offset between two link pairs
     */
    void setLinkPairThreshold(const double linkPair_threshold);

    /**
     * @brief setDetectionThreshold set _Detection_threshold
     * @param detection_threshold (always positive), the maximum distance which we use to look for link pairs
     */
    void setDetectionThreshold(const double detection_threshold);

    /**
     * @brief update recomputes Aineq and bUpperBound if x is different than the
     *  previously stored value
     * @param x the state vector.
     */
    void update();


    /**
     * @brief setCollisionList permits to set a list of collision pairs (links).
     * @note when using set CollisionList, the internal list of collision is reset. This entails the resizing of
     * internal constraint matrices. For this reason, this method SHOULD NOT be called once the constraint has been
     * added into the solver but only BEFORE!
     * @param collisionList set of link pairs names
     * @return false if links are not founded in the model
     */
    bool setCollisionList(std::set<std::pair<std::string, std::string>> collisionList);

    /**
     * @brief updateCollisionList permits to update the collision list with different collision pairs
     * @note the parameter collisionList in input can not be larger than the maximum number of link pairs (_max_pairs),
     * this is to avoid resize of constraint matrix
     * @param collisionList set of link pairs names
     * @return false if collisionList.size() > _max_pairs or if links are not founded in the model
     */
    bool updateCollisionList(std::set<std::pair<std::string, std::string>> collisionList);

    /**
     * @brief collisionModelUpdated must be called after a collision has been added or
     * removed from the model
     */
    void collisionModelUpdated();

    // TODO: waiting for world collision support !
    //
    // /**
    //  * @brief add/remove world collision objects according to the given planning
    //  * scene world
    //  * @return true if all requests (additions, deletions) could be performs
    //  * succesfully, false on (partial) insuccess
    //  */
    // bool setWorldCollisions(const moveit_msgs::PlanningSceneWorld& wc);

    // /**
    //  * @brief add single collision to the world
    //  * @param id is the unique collision id
    //  * @param fcl_obj is the fcl collision object (geometry + transform)
    //  * @return true if input is valid
    //  */
    // bool addWorldCollision(const std::string& id,
    //                        std::shared_ptr<fcl::CollisionObjectd> fcl_obj);

    // /**
    //  * @brief remove world collision with given id
    //  */
    // bool removeWorldCollision(const std::string& id);

    // /**
    //  * @brief change the transform w.r.t. the world for the given
    //  * world collision
    //  */
    // bool moveWorldCollision(const std::string& id,
    //                         Eigen::Affine3d new_pose);

    /**
     * @brief setBoundScaling sets bound scaling for the capsule constraint
     * @param boundScaling is a number which should be lower than 1.0
     *        (e.g. 1./2. means we are looking two steps ahead and will avoid
     *         collision with the capsule by slowing down)
     */
    void setBoundScaling(const double boundScaling);

    /**
     * @brief setLinksVsEnvironment
     * @param links
     */
    void setLinksVsEnvironment(const std::list<std::string>& links);

    /**
     * @brief getter for the internal collision model
     */
    XBot::Collision::CollisionModel& getCollisionModel();

    /**
     * @brief getter for the internal collision model
     */
    const XBot::Collision::CollisionModel& getCollisionModel() const;

    /**
     * @brief get the vector of witness points used by this constraint
     * during the last call to update(), in ascending distance order
     * @param wp
     */
    void getOrderedWitnessPointVector(WitnessPointVector& wp) const;

    /**
     * @brief getOrderedLinkPairVector
     * @param lp
     */
    void getOrderedLinkPairVector(LinkPairVector& lp) const;

    /**
     * @brief getOrderedDistanceVector
     * @param d
     */
    void getOrderedDistanceVector(std::vector<double>& d) const;

    ~CollisionAvoidance();

protected:

    /**
     * @brief _include_env
     */
    bool _include_env;

    /**
     * @brief _bound_scaling
     */
    double _bound_scaling;

    /**
     * @brief _distance_threshold is the allowable minimum distance between every Link pair
     */
    double _distance_threshold;

    /**
     * @brief all the link pairs whose minimum distance are smaller than this "_Detection_threshold" would be dealt with further
     */
    double _detection_threshold;

    /**
     * @brief _max_pairs
     */
    int _max_pairs;

    /**
     * @brief _robot
     */
    const XBot::ModelInterface& _robot;

    XBot::ModelInterface::Ptr _collision_model;

    /**
     * @brief _dist_calc
     */
    std::unique_ptr<XBot::Collision::CollisionModel> _dist_calc;

    Eigen::VectorXd _distances;
    Eigen::MatrixXd _distance_J;
    int _num_active_pairs;
    mutable WitnessPointVector _wpv;
    mutable LinkPairVector _lpv;

    /**
     * @brief _Jtmp
     */
    Eigen::MatrixXd _Jtmp;

};

} } }


#endif // COLLISIONAVOIDANCE_H

