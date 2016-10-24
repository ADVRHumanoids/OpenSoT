#ifndef _LEGACY_BOUNDS_VELOCITY_COMVELOCITY_H__
#define _LEGACY_BOUNDS_VELOCITY_COMVELOCITY_H__


 #include <OpenSoT/constraints/velocity/CoMVelocity.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            class CoMVelocity: public OpenSoT::constraints::velocity::CoMVelocity {
            public:
                CoMVelocity(const yarp::sig::Vector velocityLimits,
                            const double dT,
                            const yarp::sig::Vector& x,
                            iDynUtils& robot):
                    OpenSoT::constraints::velocity::CoMVelocity(cartesian_utils::toEigen(velocityLimits),
                    dT, cartesian_utils::toEigen(x), robot){}

                virtual void update(const yarp::sig::Vector &x)
                {
                    OpenSoT::constraints::velocity::CoMVelocity::update(cartesian_utils::toEigen(x));
                }

                yarp::sig::Vector getVelocityLimits()
                {
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::constraints::velocity::CoMVelocity::getVelocityLimits());
                }

                void setVelocityLimits(const yarp::sig::Vector velocityLimits)
                {
                    OpenSoT::constraints::velocity::CoMVelocity::setVelocityLimits(
                                cartesian_utils::toEigen(velocityLimits));
                }
            };
        }
        }
    }
 }

#endif
