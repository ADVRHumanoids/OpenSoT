#ifndef _LEGACY_TASKS_VELOCITY_MANIPULABILITY_H__
#define _LEGACY_TASKS_VELOCITY_MANIPULABILITY_H__

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>

using namespace yarp::math;


namespace OpenSoT {
namespace legacy{
    namespace tasks {
        namespace velocity {
            class Manipulability : public OpenSoT::tasks::velocity::Manipulability {
            public:
                void setW(const yarp::sig::Matrix& W){
                    OpenSoT::tasks::velocity::Manipulability::setW(
                                cartesian_utils::toEigen(W));
                }

                yarp::sig::Matrix getW(){
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::tasks::velocity::Manipulability::getW());
                }


            };
        }
    }
    }
}

#endif
