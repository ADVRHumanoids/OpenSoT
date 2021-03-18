#include <OpenSoT/tasks/velocity/CollisionRepulsiveField.h>
#include <OpenSoT/utils/collision_utils.h>


using namespace OpenSoT::tasks::velocity;
using namespace XBot;

namespace
{
Eigen::Vector3d k2e(const KDL::Vector &k)
{
    return Eigen::Vector3d(k[0], k[1], k[2]);
}
}

CollisionRepulsiveField::CollisionRepulsiveField(const Eigen::VectorXd &x,
                                                 const ModelInterface &model,
                                                 int max_pairs):
    Task("collision_repulsive", x.size()),
    _model(model),
    _d_th(0.01),
    _max_pairs(max_pairs)
{
    // construct link distance computation util
    _dist = std::make_unique<ComputeLinksDistance>(model);

    // initialize structures and set weight
    _A.setZero(_max_pairs, getXSize());
    _b.setZero(_max_pairs);
    _W.setIdentity(_A.size(), _A.size());
}

void CollisionRepulsiveField::setDistanceThreshold(double d_th)
{
    _d_th = d_th;
}

double CollisionRepulsiveField::getDistanceThreshold() const
{
    return _d_th;
}

int CollisionRepulsiveField::getMaxLinkPairs() const
{
    return _max_pairs;
}

bool CollisionRepulsiveField::setWhiteList(std::list<std::pair<std::string, std::string> > pairs)
{
    return _dist->setCollisionWhiteList(pairs);
}

void CollisionRepulsiveField::_update(const Eigen::VectorXd &x)
{
    _A.setZero(_max_pairs, getXSize());
    _b.setZero(_max_pairs);

    double d_hi = _d_th;
    double d_lo = d_hi * 0.1;

    auto distance_list = _dist->getLinkDistances(d_hi);

    int row_idx = 0;

    for(const auto& data : distance_list)
    {
        // closest point on first link
        Eigen::Vector3d p1_local = k2e(data.getClosestPoints().first.p);

        // closest point on second link
        Eigen::Vector3d p2_local = k2e(data.getClosestPoints().second.p);

        // global closest points
        Eigen::Affine3d w_T_l1;
        _model.getPose(data.getLinkNames().first, w_T_l1);
        Eigen::Vector3d p1_world = w_T_l1*p1_local;

        Eigen::Affine3d w_T_l2;
        _model.getPose(data.getLinkNames().second, w_T_l2);
        Eigen::Vector3d p2_world = w_T_l2*p2_local;

        // minimum distance direction
        Eigen::Vector3d p12 = p2_world - p1_world;

        // minimum distance regularized
        double d12 = p12.norm() + 1e-6;

        // jacobian of p1
        _model.getJacobian(data.getLinkNames().first,
                           p1_local,
                           _Jtmp);

        _A.row(row_idx) = -p12.transpose() * _Jtmp.topRows<3>() / d12;

        // jacobian of p2
        _model.getJacobian(data.getLinkNames().second,
                           p2_local,
                           _Jtmp);

        _A.row(row_idx) += p12.transpose() * _Jtmp.topRows<3>() / d12;

        _b(row_idx) = getLambda()*(d_hi - d12);

        ++row_idx;
    }

}

CollisionRepulsiveField::~CollisionRepulsiveField() = default;


