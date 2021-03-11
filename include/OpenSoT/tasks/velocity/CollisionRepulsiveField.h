#ifndef OPENSOT_TASK_VEL_COLLISIONREPULSIVEFIELD_H
#define OPENSOT_TASK_VEL_COLLISIONREPULSIVEFIELD_H

#include <OpenSoT/Task.h>

// forward declaration to avoid users pulling in all fcl headers
class ComputeLinksDistance;

namespace OpenSoT { namespace tasks { namespace velocity {

class CollisionRepulsiveField : public Task<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    typedef boost::shared_ptr<CollisionRepulsiveField> Ptr;

    CollisionRepulsiveField(const Eigen::VectorXd& x,
                            const XBot::ModelInterface& model,
                            int max_pairs);

    void setDistanceThreshold(double d_th);

    double getDistanceThreshold() const;

    int getMaxLinkPairs() const;

    bool setWhiteList(std::list<std::pair<std::string, std::string>> pairs);

    ~CollisionRepulsiveField() override;

private:

    std::unique_ptr<ComputeLinksDistance> _dist;

    void _update(const Eigen::VectorXd& x) override;

    const XBot::ModelInterface& _model;

    double _d_th;

    int _max_pairs;

    Eigen::MatrixXd _Jtmp;
};


} } }

#endif // COLLISIONREPULSIVEFIELD_H
