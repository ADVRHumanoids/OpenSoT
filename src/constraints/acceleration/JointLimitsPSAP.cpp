#include <OpenSoT/constraints/acceleration/JointLimitsPSAP.h>

using namespace OpenSoT::constraints::acceleration;

JointLimitsPSAP::JointLimitsPSAP(XBot::ModelInterface &robot,
                                 const AffineHelper &qddot,
                                 const Eigen::VectorXd &jointBoundMax,
                                 const Eigen::VectorXd &jointBoundMin,
                                 const Eigen::VectorXd &jointVelMax,
                                 const Eigen::VectorXd &jointAccMax,
                                 const double dt):
    Constraint("joint_limits_psap", qddot.getInputSize()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _jointVelMax(jointVelMax),
    _jointAccMax(jointAccMax),
    _robot(robot),
    _dt(dt),
    _p(1.)
{
    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                Eigen::VectorXd::Zero(jointBoundMin.size()),
                Eigen::VectorXd::Zero(jointBoundMax.size()),
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    __upperBound = jointAccMax;
    __lowerBound = -jointAccMax;

    update(Eigen::VectorXd(1));
}

void JointLimitsPSAP::update(const Eigen::VectorXd &x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);

    //Position:
    // qnext = q + qdot*p*dt + qddot*p*dt*p*dt
    // qmin <= qnext <= qmax --> (qmin -q -qdot*p*dt)/(p*dt*p*dt) <= qddot <= (qmax -q -qdot*p*dt)/(p*dt*p*dt)

    _pmax = (_jointLimitsMax -_q -_qdot*_p*_dt)/(_p*_dt*_p*_dt);
    _pmin = (_jointLimitsMin -_q -_qdot*_p*_dt)/(_p*_dt*_p*_dt);

    //Velocity
    // qdotnext = qdot + qddot*p*dt
    // qdotmin <= qdotnext <= qdotmax --> (qdotmin -qdot)/(p*dt) <= qddot <= (qdotmax -qdot)/(p*dt)

    _vmax = (_jointVelMax -_qdot)/(_p*_dt);
    _vmin = (-_jointVelMax -_qdot)/(_p*_dt);

    for(unsigned int i = 0; i < _jointAccMax.size(); ++i)
    {
        __upperBound[i] = std::min(std::min(_pmax[i], _vmax[i]), _jointAccMax[i]);
        __lowerBound[i] = std::max(std::max(_pmin[i], _vmin[i]), -_jointAccMax[i]);

        if(__upperBound[i] < __lowerBound[i])
        {
            double ub = __upperBound[i];
            __upperBound[i] = __lowerBound[i];
            __lowerBound[i] = ub;
        }

        if(__lowerBound[i] < -_jointAccMax[i])
            __lowerBound[i] = -_jointAccMax[i];

        if(__upperBound[i] > _jointAccMax[i])
            __upperBound[i] = _jointAccMax[i];
    }


    _generic_constraint_internal->setBounds(__upperBound, __lowerBound);

    _generic_constraint_internal->update(x);

    _Aineq = _generic_constraint_internal->getAineq();
    _bLowerBound = _generic_constraint_internal->getbLowerBound();
    _bUpperBound = _generic_constraint_internal->getbUpperBound();
}

bool JointLimitsPSAP::setPStepAheadPredictor(const double p)
{
    if(p < 1.)
        return false;
    _p = p;
    return true;
}
