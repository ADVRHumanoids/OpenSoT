
#ifndef _LEGACY_TASKS_VELOCITY_COM_H_
#define _LEGACY_TASKS_VELOCITY_COM_H_

#include <OpenSoT/tasks/velocity/CoM.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>



/**
  * @example example_com.cpp
  * The CoM class implements a task that tries to impose a position
  * of the CoM w.r.t. the support foot.
  */
 namespace OpenSoT {
 namespace legacy{
    namespace tasks {
        namespace velocity {

        class CoM : public OpenSoT::tasks::velocity::CoM {
        public:
            CoM(const yarp::sig::Vector& x,
                    iDynUtils& robot):
                OpenSoT::tasks::velocity::CoM(cartesian_utils::toEigen(x), robot)
            {
            }

            void setReference(const yarp::sig::Vector& desiredPosition)
            {
                OpenSoT::tasks::velocity::CoM::setReference(cartesian_utils::toEigen(desiredPosition));
            }

            void setReference(const yarp::sig::Vector& desiredPosition,
                                  const yarp::sig::Vector& desiredVelocity)
            {
                OpenSoT::tasks::velocity::CoM::setReference(
                            cartesian_utils::toEigen(desiredPosition),
                            cartesian_utils::toEigen(desiredVelocity));
            }


            yarp::sig::Vector getReference()
            {
                return cartesian_utils::fromEigentoYarp(
                            OpenSoT::tasks::velocity::CoM::getReference());
            }

            void getReference(yarp::sig::Vector& desiredPosition,
                                  yarp::sig::Vector& desiredVelocity)
            {
                Eigen::VectorXd a,b;
               OpenSoT::tasks::velocity::CoM::getReference(a,b);
               desiredPosition = cartesian_utils::fromEigentoYarp(a);
               desiredVelocity = cartesian_utils::fromEigentoYarp(b);
            }


            yarp::sig::Vector getActualPosition()
            {
                return cartesian_utils::fromEigentoYarp(
                       OpenSoT::tasks::velocity::CoM::getActualPosition());
            }

             yarp::sig::Vector getError()
             {
                 return cartesian_utils::fromEigentoYarp(
                        OpenSoT::tasks::velocity::CoM::getError());
             }


            };
        }
        }
    }
 }

#endif
