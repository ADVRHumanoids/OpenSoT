#ifndef __TASKS_MINIMIZE_ACCELERATION_H__
#define __TASKS_MINIMIZE_ACCELERATION_H__

 #include <OpenSoT/Task.h>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 namespace OpenSoT {
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Minimize Acceleration class implements a task that tries to minimize the change in velocity.
             */
            class MinimizeAcceleration : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<MinimizeAcceleration> Ptr;
            protected:
                yarp::sig::Vector _x_before;

            public:

                MinimizeAcceleration(const yarp::sig::Vector& x);

                ~MinimizeAcceleration();

                void setLambda(double lambda);

                void _update(const yarp::sig::Vector& x);
            };
        }
    }
 }

#endif
