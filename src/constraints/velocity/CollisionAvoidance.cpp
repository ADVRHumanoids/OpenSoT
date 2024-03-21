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
        const XBot::ModelInterface& robot,
        int max_pairs,
        urdf::ModelConstSharedPtr collision_urdf,
        srdf::ModelConstSharedPtr collision_srdf):
    Constraint("self_collision_avoidance", robot.getNv()),
    _detection_threshold(std::numeric_limits<double>::max()),
    _distance_threshold(0.001),
    _robot(robot),
    _bound_scaling(1.0),
    _max_pairs(max_pairs)
{
    // construct custom model for collisions
    _collision_model =
        ModelInterface::getModel(collision_urdf ? collision_urdf : robot.getUrdf(),
                                 collision_srdf ? collision_srdf : robot.getSrdf(),
                                 robot.getType());

    // construct link distance computation util
    _dist_calc = std::make_unique<Collision::CollisionModel>(_collision_model);

    // if max pairs not specified, set it as number of total
    // link pairs
    if(_max_pairs < 0)
    {
        // note: threshold = inf
        _max_pairs = _dist_calc->getNumCollisionPairs();
    }
    _distance_J.setZero(_dist_calc->getNumCollisionPairs(), getXSize());

    // initialize matrices
    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setZero(_max_pairs);
    _bLowerBound.setConstant(_max_pairs, std::numeric_limits<double>::lowest());

    // initialize link pair vector
    _lpv = _dist_calc->getCollisionPairs(_include_env);

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
    // update collision model
    //_collision_model->syncFrom(_robot, ControlMode::POSITION); <-- not updating
    _collision_model->setJointPosition(_robot.getJointPosition());
    _collision_model->update();
    _dist_calc->update();

    // reset constraint
    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setConstant(_max_pairs, std::numeric_limits<double>::max());

    // compute distances
    _dist_calc->computeDistance(_distances, _detection_threshold);

    // compute jacobians
    _dist_calc->getDistanceJacobian(_distance_J);

    // populate Aineq and bUpperBound
    for(int i : _dist_calc->getOrderedCollisionPairIndices())
    {
        if(i < _max_pairs)
        {
            _Aineq.row(i) = -_distance_J.row(i);
            _bUpperBound(i) = _bound_scaling*(_distances(i) - _distance_threshold);

            // to avoid infeasibilities, cap upper bound to zero
            // (i.e. don't change current distance if in collision)
            if(_bUpperBound(i) < 0.0)
            {
                _bUpperBound(i) = 0.0;
            }
        }
        else
            break;

    }

}

bool CollisionAvoidance::setCollisionList(std::set<std::pair<std::string, std::string>> collisionList)
{
    try
    {
        _dist_calc->setLinkPairs(collisionList);
        _max_pairs = collisionList.size();
        // initialize matrices
        _Aineq.setZero(_max_pairs, getXSize());
        _bUpperBound.setZero(_max_pairs);
        _bLowerBound.setConstant(_max_pairs, std::numeric_limits<double>::lowest());
        _distance_J.setZero(_max_pairs, _robot.getNv());

        return true;
    }
    catch(std::out_of_range& e)
    {
        Logger::error(e.what());
        return false;
    }
}

bool CollisionAvoidance::updateCollisionList(std::set<std::pair<std::string, std::string>> collisionList)
{
    if(collisionList.size() > _max_pairs)
    {
        Logger::error("collisionList.size() > _max_pairs when updating collision list");
        return false;
    }

    try
    {
        _dist_calc->setLinkPairs(collisionList);
        _Aineq.setZero();
        _bUpperBound.setZero();
        _distance_J.setZero(collisionList.size(), _robot.getNv());
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

void CollisionAvoidance::setLinksVsEnvironment(const std::list<std::string> &links)
{
    _dist_calc->setLinksVsEnvironment(std::set(links.begin(), links.end()));
}

Collision::CollisionModel &CollisionAvoidance::getCollisionModel()
{
    return *_dist_calc;
}

const Collision::CollisionModel &CollisionAvoidance::getCollisionModel() const
{
    return *_dist_calc;
}

void CollisionAvoidance::getOrderedWitnessPointVector(WitnessPointVector &wp) const
{
    wp.clear();

    _dist_calc->getWitnessPoints(_wpv, _include_env);

    const auto& ordered_idx = _dist_calc->getOrderedCollisionPairIndices();

    for(int i = 0; i < _num_active_pairs && i < _distances.size(); i++)
    {
        wp.push_back(_wpv[ordered_idx[i]]);
    }
}

void CollisionAvoidance::getOrderedLinkPairVector(LinkPairVector &lp) const
{
    lp.clear();

    const auto& ordered_idx = _dist_calc->getOrderedCollisionPairIndices();

    for(int i = 0; i < _num_active_pairs && i < _distances.size(); i++)
    {
        lp.push_back(_lpv[ordered_idx[i]]);
    }
}

void CollisionAvoidance::getOrderedDistanceVector(std::vector<double> &d) const
{
    d.clear();

    const auto& ordered_idx = _dist_calc->getOrderedCollisionPairIndices();

    for(int i = 0; i < _num_active_pairs && i < _distances.size(); i++)
    {
        d.push_back(_distances[ordered_idx[i]]);
    }
}

CollisionAvoidance::~CollisionAvoidance() = default;

