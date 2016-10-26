#ifndef _LEGACY_TASKS_VELOCITY_MINIMUMVELOCITY_H_
#define _LEGACY_TASKS_VELOCITY_MINIMUMVELOCITY_H_

 #include <OpenSoT/tasks/velocity/MinimumVelocity.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

/**
 * @example example_MinimumVelocity.cpp
 * The MinimumVelocity class implements a task that tries to minimize joints velocities.
 */

 namespace OpenSoT {
 namespace legacy{
    namespace tasks {
        namespace velocity {
            /**
             * @brief The MinimumVelocity class implements a task that tries to minimize joints velocities.
             * Notice that you can implement task space minimum velocity tasks by setting lambda to 0 in the corresponding tasks, i.e.:
                MinimumCOMVelocity
                MinimumCartesianVelocity
             * You can see an example of it in @ref example_MinimumVelocity.cpp
             */
            class MinimumVelocity : public OpenSoT::tasks::velocity::MinimumVelocity {
            public:


            };
        }
        }
    }
 }

#endif
