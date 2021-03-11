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
    _dist = std::make_unique<ComputeLinksDistance>(const_cast<ModelInterface&>(model));

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

    double d_hi = 0.01;
    double d_lo = 0.001;

    auto distance_list = _dist->getLinkDistances(d_hi);

    int row_idx = 0;

    for(const auto& data : distance_list)
    {
        // we filled the task, skip the rest of colliding pairs
        if(row_idx >= _max_pairs)
        {
            break;
        }

        // closest point on first link
        Eigen::Vector3d p1 = k2e(data.getLink_T_closestPoint().first.p);

        // closest point on second link
        Eigen::Vector3d p2 = k2e(data.getLink_T_closestPoint().second.p);

        // minimum distance direction
        Eigen::Vector3d p12 = p2 - p1;

        // minimum distance regularized
        double d12 = p12.norm() + 1e-6;

        // jacobian of p1
        _model.getJacobian(data.getLinkNames().first,
                           p1,
                           _Jtmp);

        _A.row(row_idx) = -p12.transpose() * _Jtmp / d12;

        // jacobian of p2
        _model.getJacobian(data.getLinkNames().second,
                           p2,
                           _Jtmp);

        _A.row(row_idx) += p12.transpose() * _Jtmp / d12;

        // task objective
        if(d12 >= d_lo)
        {
            _b(row_idx) = 0;
        }
        else
        {
            _b(row_idx) = getLambda()*(d_hi - d12);
        }

        ++row_idx;
    }

}

CollisionRepulsiveField::~CollisionRepulsiveField() = default;


