#include <OpenSoT/constraints/acceleration/VelocityLimits.h>
#include <memory>

OpenSoT::constraints::acceleration::VelocityLimits::VelocityLimits(XBot::ModelInterface& robot,
                                                   const AffineHelper& qddot,
                                                   const double qDotLimit,
                                                   const double dT):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >("velocity_limits", qddot.getInputSize()),
    _robot(robot),
    _dT(dT),
    _p(1.)
{
    _qdotmin.setOnes(qddot.getOutputSize());
    _qdotmin *= -qDotLimit;
    _qdotmax.setOnes(qddot.getOutputSize());
    _qdotmax *= qDotLimit;

    _qdot.setZero(_qdotmax.size());

    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                ( _qdotmax - _qdot)/(_dT*_p),
                ( _qdotmin - _qdot)/(_dT*_p),
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);
    update(Eigen::VectorXd(0));
}

OpenSoT::constraints::acceleration::VelocityLimits::VelocityLimits(XBot::ModelInterface& robot,
               const AffineHelper& qddot,
               const Eigen::VectorXd& qDotLimit,
               const double dT):
    Constraint< Eigen::MatrixXd, Eigen::VectorXd >("velocity_limits", qddot.getInputSize()),
    _robot(robot),
    _dT(dT),
    _p(1.)
{
    _qdotmin = -qDotLimit;
    _qdotmax = qDotLimit;

    _qdot.setZero(_qdotmax.size());

    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                ( _qdotmax - _qdot)/(_dT*_p),
                ( _qdotmin - _qdot)/(_dT*_p),
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);
    update(Eigen::VectorXd(0));
}

void OpenSoT::constraints::acceleration::VelocityLimits::update(const Eigen::VectorXd &x)
{
    _robot.getJointVelocity(_qdot);

    _generic_constraint_internal->setBounds(
                ( _qdotmax - _qdot)/(_dT*_p),
                ( _qdotmin - _qdot)/(_dT*_p)
                );

    _generic_constraint_internal->update(x);

    _Aineq = _generic_constraint_internal->getAineq();
    _bLowerBound = _generic_constraint_internal->getbLowerBound();
    _bUpperBound = _generic_constraint_internal->getbUpperBound();
}

void OpenSoT::constraints::acceleration::VelocityLimits::setVelocityLimits(const double qDotLimit)
{
    _qdotmax.setOnes(_qdotmax.size());
    _qdotmax *= qDotLimit;

    _qdotmin.setOnes(_qdotmin.size());
    _qdotmin *= -qDotLimit;
}

void OpenSoT::constraints::acceleration::VelocityLimits::setVelocityLimits(const Eigen::VectorXd& qDotLimit)
{
    _qdotmin = -qDotLimit;
    _qdotmax = qDotLimit;
}

bool OpenSoT::constraints::acceleration::VelocityLimits::setPStepAheadPredictor(const double p)
{
    if(p < 1.)
        return false;
    _p = p;
    return true;
}
