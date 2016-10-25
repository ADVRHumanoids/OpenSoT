#ifndef _LEGACY_BOUNDS_TASKTOCONSTRAINT_H__
#define _LEGACY_BOUNDS_TASKTOCONSTRAINT_H__

#include <OpenSoT/constraints/TaskToConstraint.h>

#include <yarp/sig/all.h>
#include <list>


 namespace OpenSoT {
 namespace legacy {

    namespace constraints {

        /**
         * @brief The TaskToConstraint class transforms a task into an equality constraint:
         *        \f$ A*x = b \f$
         * BilateralConstraint:
         *   \f$ b <= A*x <= b \f$
         */
        class TaskToConstraint: public OpenSoT::constraints::TaskToConstraint {
        public:

            void update(const yarp::sig::Vector &q);


        };
    }
    }
 }

#endif
