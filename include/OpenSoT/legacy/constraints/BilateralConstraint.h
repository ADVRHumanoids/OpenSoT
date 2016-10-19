#ifndef _LEGACY_BOUNDS_BILATERALCONSTRAINT_H_
#define _LEGACY_BOUNDS_BILATERALCONSTRAINT_H_

#include <OpenSoT/Constraint.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <yarp/sig/all.h>
#include <list>
#include <idynutils/cartesian_utils.h>


 namespace OpenSoT {
 namespace legacy{
    namespace constraints {

        class BilateralConstraint: public OpenSoT::constraints::BilateralConstraint {

        public:
            BilateralConstraint(const yarp::sig::Matrix &Aineq,
                                const yarp::sig::Vector &bLowerBound,
                                const yarp::sig::Vector &bUpperBound)
            {
                OpenSoT::constraints::BilateralConstraint(cartesian_utils::toEigen(Aineq),
                        cartesian_utils::toEigen(bLowerBound), cartesian_utils::toEigen(bUpperBound));
            }

            BilateralConstraint(const std::string constraintName,
                                const yarp::sig::Matrix &Aineq,
                                const yarp::sig::Vector &bLowerBound,
                                const yarp::sig::Vector &bUpperBound)
            {
                OpenSoT::constraints::BilateralConstraint(constraintName,
                    cartesian_utils::toEigen(Aineq),
                    cartesian_utils::toEigen(bLowerBound), cartesian_utils::toEigen(bUpperBound));
            }

        };
    }
 }
 }

#endif
