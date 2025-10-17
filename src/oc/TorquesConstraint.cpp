#include <OpenSoT/oc/TorquesConstraint.h>

using namespace OpenSoT::oc;

DynamicsConstraint::DynamicsConstraint(XBot::ModelInterface &robot,
                                       const AffineHelper &dX,
                                       const AffineHelper &dU) : Constraint("DynamicsConstraint", robot.getNv()),
                                                                 _robot(robot),
                                                                 _dU(dU),
                                                                 _dX(dX),
                                                                 _task(robot, dX, dU)
{

    update();
}

void DynamicsConstraint::_update()
{
    _task.update();

    _Aineq = _task.getA();

    _bLowerBound = -_robot.getEffortLimits() + _task.getb(); // + bcause of the deffinition inside the task
    _bUpperBound = _robot.getEffortLimits() + _task.getb();  // + bcause of the deffinition inside the task
}

