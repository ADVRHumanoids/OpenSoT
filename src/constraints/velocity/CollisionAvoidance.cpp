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

#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <xbot2_interface/collision.h>

using namespace OpenSoT::constraints::velocity;
using namespace XBot;

CollisionAvoidance::CollisionAvoidance(
        const Eigen::VectorXd& x,
        const XBot::ModelInterface& robot,
        int max_pairs,
        urdf::ModelConstSharedPtr collision_urdf,
        srdf::ModelConstSharedPtr collision_srdf):
    Constraint("self_collision_avoidance", robot.getNv()),
    _detection_threshold(std::numeric_limits<double>::max()),
    _distance_threshold(0.001),
    _robot(robot),
    _x_cache(x),
    _bound_scaling(1.0),
    _max_pairs(max_pairs)
{
    // construct custom model for collisions
    ModelInterface::Ptr collision_model = ModelInterface::getModel(collision_urdf, collision_srdf, robot.getType());

    // construct link distance computation util
    _dist_calc = std::make_unique<Collision::CollisionModel>(collision_model);

    // if max pairs not specified, set it as number of total
    // link pairs
    if(_max_pairs < 0)
    {
        // note: threshold = inf
        _max_pairs = _dist_calc->getNumCollisionPairs();
    }

    // initialize matrices
    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setZero(_max_pairs);
    _bLowerBound.setConstant(_max_pairs, std::numeric_limits<double>::lowest());

}

double CollisionAvoidance::getLinkPairThreshold()
{
    return _distance_threshold;
}

double CollisionAvoidance::getDetectionThreshold()
{
    return _detection_threshold;
}

void CollisionAvoidance::setLinkPairThreshold(const double linkPair_threshold)
{
    _distance_threshold = std::fabs(linkPair_threshold);
}

void CollisionAvoidance::setDetectionThreshold(const double detection_threshold)
{
    _detection_threshold = std::fabs(detection_threshold);
}

void CollisionAvoidance::update(const Eigen::VectorXd &x)
{

    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setConstant(_max_pairs, std::numeric_limits<double>::max());

    // compute distances
    _dist_calc->computeDistance(_distances, _detection_threshold);

    // compute jacobians
    _dist_calc->getDistanceJacobian(_distance_J);

    // order indices in ascending distance
    std::multimap<double, int> dist_to_idx;
    for(int i = 0; i < _distances.size(); i++)
    {
        dist_to_idx.insert({_distances[i], i});
    }

    // populate Aineq and bUpperBound
    int row_idx = 0;
    for(auto& [d, i] : dist_to_idx)
    {
        _Aineq.row(row_idx) = _distance_J.row(i);
        _bUpperBound(row_idx) = _bound_scaling*(_distances(i) - _distance_threshold);

        // to avoid infeasibilities, cap upper bound to zero
        // (i.e. don't change current distance if in collision)
        if(_bUpperBound(row_idx) < 0.0)
        {
            _bUpperBound(row_idx) = 0.0;
        }

        ++row_idx;
    }

}

bool CollisionAvoidance::setCollisionWhiteList(std::set<std::pair<std::string, std::string>> whiteList)
{
    try
    {
        _dist_calc->setLinkPairs(whiteList);
        return true;
    }
    catch(std::out_of_range& e)
    {
        Logger::error(e.what());
        return false;
    }
}

// bool CollisionAvoidance::setWorldCollisions(const moveit_msgs::PlanningSceneWorld &wc)
// {
//     return _dist_calc->setWorldCollisions(wc);
// }

// void CollisionAvoidance::setLinksVsEnvironment(const std::list<std::string>& links)
// {
//     _dist_calc->setLinksVsEnvironment(links);
// }

// bool CollisionAvoidance::addWorldCollision(const std::string &id,
//                                                std::shared_ptr<fcl::CollisionObjectd> fcl_obj)
// {
//     return _dist_calc->addWorldCollision(id, fcl_obj);
// }

// bool CollisionAvoidance::removeWorldCollision(const std::string &id)
// {
//     return _dist_calc->removeWorldCollision(id);
// }

// bool CollisionAvoidance::moveWorldCollision(const std::string &id,
//                                             Eigen::Affine3d new_pose)
// {
//     return _dist_calc->moveWorldCollision(id, new_pose);
// }

void CollisionAvoidance::setBoundScaling(const double boundScaling)
{
    _bound_scaling = boundScaling;
}

Collision::CollisionModel &CollisionAvoidance::getCollisionModel()
{
    return *_dist_calc;
}

const Collision::CollisionModel &CollisionAvoidance::getCollisionModel() const
{
    return *_dist_calc;
}

CollisionAvoidance::~CollisionAvoidance() = default;

