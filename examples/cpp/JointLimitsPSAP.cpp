#include "JointLimitsPSAP.h"

using namespace OpenSoT::constraints::acceleration;

JointLimitsPSAP::JointLimitsPSAP(XBot::ModelInterface &robot,
                                 const AffineHelper &qddot,
                                 const Eigen::VectorXd &jointBoundMax,
                                 const Eigen::VectorXd &jointBoundMin,
                                 const Eigen::VectorXd &jointVelMax,
                                 const Eigen::VectorXd &jointAccMax,
                                 const double dt):
    /**
      * @brief Constraint needs to be initialized with an id and size of the problem, this size can be retrieved from
      * the variable (which in general may be DIFFERENT from the size of the considered variable!)
      */
    Constraint("joint_limits_psap", qddot.getInputSize()),
    _jointLimitsMax(jointBoundMax),
    _jointLimitsMin(jointBoundMin),
    _jointVelMax(jointVelMax),
    _jointAccMax(jointAccMax),
    _robot(robot),
    _dt(dt),
    _p(1.)
{
    /**
      *@brief in order to implement the constraint using explicit variables, we internally use a
      * GenericConstraint object that will do some computations for us
      *@note It is important to notice that in general, for a problem with accelerations and contact forces,
      * the type of constraint to be used is CONSTRAINT being applied only to a sub-part of the variables
      * (otherwise we could use as well BOUND if applied to all variables)
      */
    _generic_constraint_internal = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                "internal_generic_constraint", qddot,
                Eigen::VectorXd::Zero(jointBoundMin.size()),
                Eigen::VectorXd::Zero(jointBoundMax.size()),
                OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    /**
      * @brief Initialize some internal variables
      */
    __upperBound = jointAccMax;
    __lowerBound = -jointAccMax;

    /**
      * @brief This is the neutral configuration of the robot. In the case of a fixed base robot it may be all zeros, but in the
      * case of a floating-base one it provides a configuration which is compiant with quaternion manifold for no rotation
      */
    _zeros = _robot.getNeutralQ();

    /**
      * @brief Call the update.
      * @note The parameter is not used so we can put whatever
      */
    update();
}

void JointLimitsPSAP::update()
{
    /**
      * @brief Retrieve joint positions and velocities from internal model
      * @note The model NEEDS to be updated outside
      */
    _robot.getJointPosition(_q);
    _robot.getJointVelocity(_qdot);

    /**
     * @brief dt apply _p to control time
     */
    double dt = _p*_dt;
    /**
     * @brief Compute Position related limits:
     * qnext = q + qdot*dt + 0.5*qddot*dt^2
     * qmin <= qnext <= qmax --> (qmin -q -qdot*dt)/(0.5*dt^2) <= qddot <= (qmax -q -qdot*dt)/(0.5*dt^2)
    */
    _pmax = (_jointLimitsMax -_robot.difference(_q, _zeros) -_qdot*dt)/(0.5*dt*dt);
    _pmin = (_jointLimitsMin -_robot.difference(_q, _zeros) -_qdot*dt)/(0.5*dt*dt);

    /**
      * @brief COmpute Velocity related limits:
      * qdotnext = qdot + qddot*dt
      * qdotmin <= qdotnext <= qdotmax --> (qdotmin -qdot)/dt <= qddot <= (qdotmax -qdot)/dt
      */
    _vmax = (_jointVelMax -_qdot)/(dt);
    _vmin = (-_jointVelMax -_qdot)/(dt);

    for(unsigned int i = 0; i < _jointAccMax.size(); ++i)
    {
        /**
          * @brief upper bound and lower bounds are computed among minimum and maximum, respectively,
          * of position, velocity and acceleration limits computed on the previous steps
          */
        __upperBound[i] = std::min(std::min(_pmax[i], _vmax[i]), _jointAccMax[i]);
        __lowerBound[i] = std::max(std::max(_pmin[i], _vmin[i]), -_jointAccMax[i]);

        /**
          * @brief Check switching bounds
          */
        if(__upperBound[i] < __lowerBound[i])
        {
            double ub = __upperBound[i];
            __upperBound[i] = __lowerBound[i];
            __lowerBound[i] = ub;
        }

        /**
          * @brief Check again against maximum allowed joint accelerations
          */
        if(__lowerBound[i] < -_jointAccMax[i])
            __lowerBound[i] = -_jointAccMax[i];

        if(__upperBound[i] > _jointAccMax[i])
            __upperBound[i] = _jointAccMax[i];
    }


    /**
      *@brief set to generic constraint the computed upper and lower bounds
      */
    _generic_constraint_internal->setBounds(__upperBound, __lowerBound);

    /**
      *@brief updates the generic bounds
      */
    _generic_constraint_internal->update();

    /**
      *@brief computes final _Aineq, _bLowerBound, and _bUpperBound considering explicit variables, i.e. taking into account
      * the presence of further variables in the problem which does not appear on this constraint
      */
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
