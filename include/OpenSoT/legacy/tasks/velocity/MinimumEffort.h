
#ifndef _LEGACY_TASKS_VELOCITY_MINIMUMEFFORT_H_
#define _LEGACY_TASKS_VELOCITY_MINIMUMEFFORT_H_

 #include <OpenSoT/tasks/velocity/MinimumEffort.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/cartesian_utils.h>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>

 using namespace yarp::math;

/**
  * @example example_minimum_effort.cpp
  * The MinimumEffort class implements a task that tries to bring the robot in a minimum-effort posture.
  */
 namespace OpenSoT {
 namespace legacy{
    namespace tasks {
        namespace velocity {
            class MinimumEffort : public OpenSoT::tasks::velocity::MinimumEffort {
            public:

                MinimumEffort(const yarp::sig::Vector& x, const iDynUtils& robot_model):
                    OpenSoT::tasks::velocity::MinimumEffort(cartesian_utils::toEigen(x),
                                                            robot_model)
                {

                }


                void setW(const yarp::sig::Matrix& W){
                    OpenSoT::tasks::velocity::MinimumEffort::setW(
                                cartesian_utils::toEigen(W));
                }

                /**
                 * @brief getW get a Weight matrix for the manipulability index
                 */
                yarp::sig::Matrix getW(){
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::tasks::velocity::MinimumEffort::getW());
                }


            };
        }
    }
 }
 }

#endif
