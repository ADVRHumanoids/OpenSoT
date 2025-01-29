#ifndef OPENSOT_TASKS_COLLISIONAVOIDANCE_H
#define OPENSOT_TASKS_COLLISIONAVOIDANCE_H

#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <OpenSoT/Task.h>

namespace OpenSoT { namespace tasks { namespace velocity {

class CollisionAvoidance : public Task<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    CollisionAvoidance(constraints::velocity::CollisionAvoidance::Ptr constr);

    void _update() override;

    constraints::velocity::CollisionAvoidance::Ptr getConstraint();


private:

    constraints::velocity::CollisionAvoidance::Ptr _constr;

    Eigen::VectorXd _error;

};

} } }

#endif // COLLISIONAVOIDANCE_H
