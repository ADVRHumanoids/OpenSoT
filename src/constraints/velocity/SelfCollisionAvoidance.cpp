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

#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/utils/collision_utils.h>

using namespace OpenSoT::constraints::velocity;
using namespace XBot;

namespace
{
Eigen::Vector3d k2e(const KDL::Vector &k)
{
    return Eigen::Vector3d(k[0], k[1], k[2]);
}
}

SelfCollisionAvoidance::SelfCollisionAvoidance(
        const Eigen::VectorXd& x,
        const XBot::ModelInterface& robot,
        int max_pairs,
        urdf::ModelConstSharedPtr collision_urdf,
        srdf::ModelConstSharedPtr collision_srdf):
    Constraint("self_collision_avoidance", x.size()),
    _detection_threshold(std::numeric_limits<double>::max()),
    _distance_threshold(0.001),
    _robot(robot),
    _x_cache(x),
    _bound_scaling(1.0),
    _max_pairs(max_pairs)
{
    // construct link distance computation util
    _dist_calc = std::make_unique<ComputeLinksDistance>(robot,
                                                        collision_urdf,
                                                        collision_srdf);

    // if max pairs not specified, set it as number of total
    // link pairs
    if(_max_pairs < 0)
    {
        // note: threshold = inf
        _max_pairs = _dist_calc->getLinkDistances().size();
    }

    // initialize matrices
    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setZero(_max_pairs);
    _bLowerBound.setConstant(_max_pairs, std::numeric_limits<double>::lowest());

}

double SelfCollisionAvoidance::getLinkPairThreshold()
{
    return _distance_threshold;
}

double SelfCollisionAvoidance::getDetectionThreshold()
{
    return _detection_threshold;
}

void SelfCollisionAvoidance::setLinkPairThreshold(const double linkPair_threshold)
{
    _distance_threshold = std::fabs(linkPair_threshold);
}

void SelfCollisionAvoidance::setDetectionThreshold(const double detection_threshold)
{
    _detection_threshold = std::fabs(detection_threshold);
}

void SelfCollisionAvoidance::update(const Eigen::VectorXd &x)
{
    _Aineq.setZero(_max_pairs, getXSize());
    _bUpperBound.setConstant(_max_pairs, std::numeric_limits<double>::max());

    _distance_list = _dist_calc->getLinkDistances(_detection_threshold);

    int row_idx = 0;

    for(const auto& data : _distance_list)
    {
        // we filled the task, skip the rest of colliding pairs
        if(row_idx >= _max_pairs)
        {
            break;
        }

        // closest point on first link
        Eigen::Vector3d p1_world = k2e(data.getClosestPoints().first.p);

        // closest point on second link
        Eigen::Vector3d p2_world = k2e(data.getClosestPoints().second.p);

        // local closest points on link 1
        Eigen::Affine3d w_T_l1;
        _robot.getPose(data.getLinkNames().first, w_T_l1);
        Eigen::Vector3d p1_local = w_T_l1.inverse()*p1_world;

        // minimum distance direction
        Eigen::Vector3d p12 = p2_world - p1_world;

        // minimum distance
        double d12 = data.getDistance();

        // jacobian of p1
        _robot.getJacobian(data.getLinkNames().first,
                           p1_local,
                           _Jtmp);

        _Aineq.row(row_idx) = p12.transpose() * _Jtmp.topRows<3>() / d12;

        // jacobian of p2 only if link2 is a robot link
        // (it could be part of the environment)
        if(!data.isLink2WorldObject())
        {
            // local closest points on link 2
            Eigen::Affine3d w_T_l2;
            _robot.getPose(data.getLinkNames().second, w_T_l2);
            Eigen::Vector3d p2_local = w_T_l2.inverse()*p2_world;

            // jacobian of p2
            _robot.getJacobian(data.getLinkNames().second,
                               p2_local,
                               _Jtmp);

            _Aineq.row(row_idx) -= p12.transpose() * _Jtmp.topRows<3>() / d12;
        }

        _bUpperBound(row_idx) = _bound_scaling*(d12 - _distance_threshold);

        // to avoid infeasibilities, cap upper bound to zero
        // (i.e. don't change current distance if in collision)
        if(_bUpperBound(row_idx) < 0.0)
        {
            _bUpperBound(row_idx) = 0.0;
        }

        ++row_idx;

    }
}

bool SelfCollisionAvoidance::setCollisionWhiteList(std::list<LinkPairDistance::LinksPair> whiteList)
{
    return _dist_calc->setCollisionWhiteList(whiteList);
}

bool SelfCollisionAvoidance::setCollisionBlackList(std::list<LinkPairDistance::LinksPair> blackList)
{
    return _dist_calc->setCollisionBlackList(blackList);
}

bool SelfCollisionAvoidance::setWorldCollisions(const moveit_msgs::PlanningSceneWorld &wc)
{
    return _dist_calc->setWorldCollisions(wc);
}

void SelfCollisionAvoidance::setLinksVsEnvironment(const std::list<std::string>& links)
{
    _dist_calc->setLinksVsEnvironment(links);
}

bool SelfCollisionAvoidance::addWorldCollision(const std::string &id,
                                               boost::shared_ptr<fcl::CollisionObjectd> fcl_obj)
{
    return _dist_calc->addWorldCollision(id, fcl_obj);
}

bool SelfCollisionAvoidance::removeWorldCollision(const std::string &id)
{
    return _dist_calc->removeWorldCollision(id);
}

bool SelfCollisionAvoidance::moveWorldCollision(const std::string &id,
                                                KDL::Frame new_pose)
{
    return _dist_calc->moveWorldCollision(id, new_pose);
}

void SelfCollisionAvoidance::setBoundScaling(const double boundScaling)
{
    _bound_scaling = boundScaling;
}

SelfCollisionAvoidance::~SelfCollisionAvoidance() = default;

