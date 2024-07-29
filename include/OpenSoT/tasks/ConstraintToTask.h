#ifndef OPENSOT_CONSTRAINTTOTASK_H
#define OPENSOT_CONSTRAINTTOTASK_H

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>

namespace OpenSoT { namespace tasks {

class ConstraintToTask : public Task<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    ConstraintToTask(ConstraintPtr constraint,
                     std::function<const Eigen::VectorXd&()> );

    void _update() override;

private:

    ConstraintPtr _constr;

};
} }


#endif // CONSTRAINTTOTASK_H
