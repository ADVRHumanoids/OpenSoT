#include <OpenSoT/constraints/acceleration/JointLimitsECBF.h>

using namespace OpenSoT::constraints::acceleration;

JointLimitsECBF::JointLimitsECBF(XBot::ModelInterface &robot,
                                 const AffineHelper &qddot,
                                 const Eigen::VectorXd &jointBoundMax,
                                 const Eigen::VectorXd &jointBoundMin,
                                 const Eigen::VectorXd &jointAccMax):
    Constraint("joint_limits_ecbf", qddot.getInputSize()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _jointAccMax(jointAccMax),
    _robot(robot)
{
    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                Eigen::VectorXd::Zero(jointBoundMin.size()),
                Eigen::VectorXd::Zero(jointBoundMax.size()),
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);


    _a1.setOnes(_jointLimitsMax.size());
    _a2.setOnes(_jointLimitsMax.size());
    _ones.setOnes(_jointLimitsMax.size());

    __lowerBound = -jointAccMax;
    __upperBound = jointAccMax;

    update(Eigen::VectorXd(1));
}

void JointLimitsECBF::update(const Eigen::VectorXd &x)
{
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);

    _lower_ecbf = -(_a1 + _a2).array() * _qdot.array() -(_a1.array() * _a2.array())*(_q.array() - _jointLimitsMin.array());
    _upper_ecbf = -(_a1 + _a2).array() * _qdot.array() +(_a1.array() * _a2.array())*(_jointLimitsMax.array() - _q.array());

    for(unsigned int i = 0; i < _jointLimitsMax.size(); ++i)
    {
        __upperBound[i] = std::min(_upper_ecbf[i], _jointAccMax[i]);
        __lowerBound[i] = std::max(_lower_ecbf[i], -_jointAccMax[i]);

        if(__upperBound[i] < __lowerBound[i])
        {
            double ub = __upperBound[i];
            __upperBound[i] = __lowerBound[i];
            __lowerBound[i] = ub;
        }
    }


    _generic_constraint_internal->setBounds(__upperBound, __lowerBound);

    _generic_constraint_internal->update(x);

    _Aineq = _generic_constraint_internal->getAineq();
    _bLowerBound = _generic_constraint_internal->getbLowerBound();
    _bUpperBound = _generic_constraint_internal->getbUpperBound();
}

void JointLimitsECBF::setAlpha1(const Eigen::VectorXd &a1)
{
    _a1 = a1;
}

void JointLimitsECBF::setAlpha1(const double a1)
{
    _a1 = a1 * _ones;
}

void JointLimitsECBF::setAlpha2(const Eigen::VectorXd &a2)
{
    _a2 = a2;
}

void JointLimitsECBF::setAlpha2(const double a2)
{
    _a2 = a2 * _ones;
}


