#ifndef _LEGACY_BOUNDS_AGGREGATED_H_
#define _LEGACY_BOUNDS_AGGREGATED_H_


#include <OpenSoT/constraints/Aggregated.h>
#include <yarp/sig/all.h>
#include <boost/shared_ptr.hpp>
#include <list>
#include <idynutils/cartesian_utils.h>

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {

        class Aggregated: public OpenSoT::constraints::Aggregated {
        public:
            Aggregated(const std::list< ConstraintPtr > constraints,
                       const yarp::sig::Vector &q,
                       const unsigned int aggregationPolicy =
                            EQUALITIES_TO_INEQUALITIES |
                            UNILATERAL_TO_BILATERAL):
            OpenSoT::constraints::Aggregated(constraints, cartesian_utils::toEigen(q),
                                             aggregationPolicy)
            {

            }

            void update(const yarp::sig::Vector &x)
            {
                OpenSoT::constraints::Aggregated::update(cartesian_utils::toEigen(x));
            }

        };
    }
    }
 }

#endif
