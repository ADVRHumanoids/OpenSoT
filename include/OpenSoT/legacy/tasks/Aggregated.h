#ifndef _LEGACY_TASKS_AGGREGATED_H__
#define _LEGACY_TASKS_AGGREGATED_H__

#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>
#include <yarp/sig/all.h>
#include <boost/shared_ptr.hpp>
#include <list>


 namespace OpenSoT {
 namespace legacy{
    namespace tasks {

        class Aggregated: public OpenSoT::tasks::Aggregated {
        public:
        Aggregated(const std::list< TaskPtr > tasks,
                       const yarp::sig::Vector &q):
            OpenSoT::tasks::Aggregated(tasks, cartesian_utils::toEigen(q))
        {

        }





        };
    }

    }
 }

#endif
