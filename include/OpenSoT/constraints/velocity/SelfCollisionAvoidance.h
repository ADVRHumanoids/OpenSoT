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
#include <XBotInterface/ModelInterface.h>

#include <srdfdom/model.h>
#include <Eigen/Dense>

#include <moveit_msgs/PlanningSceneWorld.h>

class ComputeLinksDistance;

// forward declare fcl collision object
namespace fcl
{
    template <typename Scalar>
    class CollisionObject;

    using CollisionObjectd = CollisionObject<double>;
}

namespace OpenSoT { namespace constraints { namespace velocity {

/**
 * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
 *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
 *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
 *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1;
 *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
 *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
 */
class SelfCollisionAvoidance: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    typedef boost::shared_ptr<SelfCollisionAvoidance> Ptr;
    typedef std::pair<std::string, std::string> LinksPair;

    /**
     * @brief SelfCollisionAvoidance
     * @param x status
     * @param robot model
     * @param max_pairs maximum number of link pairs checked for SCA (-1 for all)
     * @param collision_urdf urdf used for collision model
     * @param collision_srdf srdf used for collision model
     * NOTE: if urdf and srdf are not specified, the one inside the robot model will be used
     */
    SelfCollisionAvoidance(const Eigen::VectorXd& x,
                           const XBot::ModelInterface& robot,
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
     * @param x the state vector. It gets cached so that we won't recompute
     * capsules distances if x didn't change
     * TODO if the capsules distances are given by a server
     * (i.e. a separate thread updaing capsules at a given rate)
     * then the inelegant caching of x is not needed anymore (notice that,
     * however, it would be nice to have a caching system
     * at the stack level so that, if two tasks in the same stack are constrained
     *  by the same constraint, this constraint
     * gets updated only once per stack update)
     */
    void update(const Eigen::VectorXd &x);


    /**
     * @brief setCollisionWhiteList resets the allowed collision matrix by setting all collision pairs as disabled.
     *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
     *        which are disabled in the SRDF
     * @param whiteList a list of links pairs for which to not check collision detection
     * @return true on success
     */
    bool setCollisionWhiteList(std::list< LinksPair > whiteList);

    /**
     * @brief setCollisionBlackList resets the allowed collision matrix by setting all collision pairs
     *        (for which collision geometries are avaiable) as enabled.
     *        It then disables all collision pairs specified in the blackList. Lastly it will disable all collision pairs
     *        which are disabled in the SRDF
     * @param blackList a list of links pairs for which to not check collision detection
     * @return true on success
     */
    bool setCollisionBlackList(std::list< LinksPair > blackList);

    /**
     * @brief add/remove world collision objects according to the given planning
     * scene world
     * @return true if all requests (additions, deletions) could be performs
     * succesfully, false on (partial) insuccess
     */
    bool setWorldCollisions(const moveit_msgs::PlanningSceneWorld& wc);

    /**
     * @brief add single collision to the world
     * @param id is the unique collision id
     * @param fcl_obj is the fcl collision object (geometry + transform)
     * @return true if input is valid
     */
    bool addWorldCollision(const std::string& id,
                           boost::shared_ptr<fcl::CollisionObjectd> fcl_obj);

    /**
     * @brief remove world collision with given id
     */
    bool removeWorldCollision(const std::string& id);

    /**
     * @brief change the transform w.r.t. the world for the given
     * world collision
     */
    bool moveWorldCollision(const std::string& id,
                            KDL::Frame new_pose);

    /**
     * @brief setBoundScaling sets bound scaling for the capsule constraint
     * @param boundScaling is a number which should be lower than 1.0
     *        (e.g. 1./2. means we are looking two steps ahead and will avoid
     *         collision with the capsule by slowing down)
     */
    void setBoundScaling(const double boundScaling);

    void setLinksVsEnvironment(const std::vector<std::string>& links);


    ~SelfCollisionAvoidance();

protected:

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

    /**
     * @brief _dist_calc
     */
    std::unique_ptr<ComputeLinksDistance> _dist_calc;

    /**
     * @brief _x_cache is a copy of last q vector used to update the constraints.
     * It is used in order to avoid multiple updated when the constraint is present
     * in multiple tasks in the same stack. It is a simple speedup waiting for the stack system to support
     * constraint caching - or for idynutils to support some form of cache system
     */
    Eigen::VectorXd _x_cache;

    /**
     * @brief _Jtmp
     */
    Eigen::MatrixXd _Jtmp;

};

} } }


#endif // SELFCOLLISIONAVOIDANCE_H

