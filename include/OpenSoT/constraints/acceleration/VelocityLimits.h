#ifndef __CONSTRAINT_ACCELERATION_VELOCITY_LIMITS_H__
#define __CONSTRAINT_ACCELERATION_VELOCITY_LIMITS_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/utils/Affine.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/constraints/GenericConstraint.h>

namespace OpenSoT { namespace constraints { namespace acceleration {

class VelocityLimits : public Constraint<Eigen::MatrixXd, Eigen::VectorXd> {

public:

    typedef std::shared_ptr<VelocityLimits> Ptr;

    VelocityLimits(XBot::ModelInterface& robot,
                   const AffineHelper& qddot,
                   const double qDotLimit,
                   const double dT);

    VelocityLimits(XBot::ModelInterface& robot,
                   const AffineHelper& qddot,
                   const Eigen::VectorXd& qDotLimit,
                   const double dT);

    virtual void update(const Eigen::VectorXd& x);

    void setVelocityLimits(const double qDotLimit);
    void setVelocityLimits(const Eigen::VectorXd& qDotLimit);

    /**
     * @brief setPStepAheadPredictor
     * @param p step predictor coefficient >= 1
     * @return false if p <1
     */
    bool setPStepAheadPredictor(const double p);

private:

    XBot::ModelInterface& _robot;
    GenericConstraint::Ptr _generic_constraint_internal;

    Eigen::VectorXd _qdot;
    Eigen::VectorXd _qdotmin, _qdotmax;

    double _dT;
    double _p;

};

} } }





#endif
