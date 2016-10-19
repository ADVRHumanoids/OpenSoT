#ifndef _LEGACY_BOUNDS_VELOCITY_JOINTLIMITS_H_
#define _LEGACY_BOUNDS_VELOCITY_JOINTLIMITS_H_

 #include <OpenSoT/constraints/velocity/JointLimits.h>

 #include <yarp/sig/all.h>
 #include <idynutils/cartesian_utils.h>

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            class JointLimits: public OpenSoT::constraints::velocity::JointLimits {
            public:
                JointLimits(const yarp::sig::Vector &q,
                            const yarp::sig::Vector &jointBoundMax,
                            const yarp::sig::Vector &jointBoundMin,
                            const double boundScaling = 1.0):
                    OpenSoT::constraints::velocity::JointLimits(cartesian_utils::toEigen(q),
                                                                cartesian_utils::toEigen(jointBoundMax),
                                                                cartesian_utils::toEigen(jointBoundMin),
                                                                boundScaling)
                {}

                void update(const yarp::sig::Vector &x)
                {
                    OpenSoT::constraints::velocity::JointLimits::update(cartesian_utils::toEigen(x));
                }

                void setBoundScaling(const double boundScaling)
                {
                    OpenSoT::constraints::velocity::JointLimits::setBoundScaling(boundScaling);
                }
            };
        }
        }
    }
 }

#endif

