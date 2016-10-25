#ifndef _LEGACY_TASKS_MINIMIZE_ACCELERATION_H_
#define _LEGACY_TASKS_MINIMIZE_ACCELERATION_H_

 #include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>
#include <idynutils/cartesian_utils.h>

 namespace OpenSoT {
 namespace legacy{
    namespace tasks {
        namespace velocity {
            class MinimizeAcceleration : public OpenSoT::tasks::velocity::MinimizeAcceleration {
            public:

                MinimizeAcceleration(const yarp::sig::Vector& x):
                    OpenSoT::tasks::velocity::MinimizeAcceleration(
                        cartesian_utils::toEigen(x))
                {

                }


            };
        }
        }
    }
 }

#endif
