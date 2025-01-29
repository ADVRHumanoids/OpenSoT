#include <OpenSoT/tasks/velocity/CollisionAvoidance.h>

using namespace OpenSoT::tasks::velocity;

OpenSoT::tasks::velocity::CollisionAvoidance::CollisionAvoidance(
    constraints::velocity::CollisionAvoidance::Ptr constr):
    Task(constr->getConstraintID() + "_task",
           constr->getXSize()),
    _constr(constr)
{
    const int task_size = _constr->getAineq().rows();
    _A.setZero(task_size, getXSize());
    _b.setZero(task_size);
    _W.setIdentity(task_size, task_size);
}

void CollisionAvoidance::_update()
{
    // update underlying constraint
    _constr->update();

    // error is good if positive
    _constr->getError(_error);

    std::cout << _error.minCoeff() << "\n";

    // compute
    _A.setZero();
    _b.setZero();

    const auto& Aineq = _constr->getAineq();

    // if error is negative, then we want DeltaE = -e
    // J*dq = -e
    // note that Aineq = -J

    for(int i = 0; i < _error.size(); i++)
    {
        if(_error(i) > 0)
        {
            continue;
        }

        _A.row(i) = Aineq.row(i);
        _b(i) = _error[i];
    }
}

OpenSoT::constraints::velocity::CollisionAvoidance::Ptr CollisionAvoidance::getConstraint()
{
    return _constr;
}
