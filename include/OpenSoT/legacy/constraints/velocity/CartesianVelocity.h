#ifndef _LEGACY_BOUNDS_VELOCITY_CARTESIANVELOCITY_H__
#define _LEGACY_BOUNDS_VELOCITY_CARTESIANVELOCITY_H__

 #include <OpenSoT/constraints/velocity/CartesianVelocity.h>
 #include <yarp/sig/all.h>


 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            class CartesianVelocity: public OpenSoT::constraints::velocity::CartesianVelocity {
            public:
                CartesianVelocity(const yarp::sig::Vector velocityLimits,
                                  const double dT,
                                  OpenSoT::tasks::velocity::Cartesian::Ptr& task):
                    OpenSoT::constraints::velocity::CartesianVelocity(
                        cartesian_utils::toEigen(velocityLimits),
                        dT, task)
                {

                }

                virtual void update(const yarp::sig::Vector &x)
                {
                    OpenSoT::constraints::velocity::CartesianVelocity::update(
                                cartesian_utils::toEigen(x));
                }

                yarp::sig::Vector getVelocityLimits()
                {
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::constraints::velocity::CartesianVelocity::getVelocityLimits());
                }

                void setVelocityLimits(const yarp::sig::Vector velocityLimits)
                {
                    OpenSoT::constraints::velocity::CartesianVelocity::setVelocityLimits(
                                cartesian_utils::toEigen(velocityLimits));
                }
            };
        }
        }
    }
 }

#endif
