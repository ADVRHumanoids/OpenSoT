#ifndef _LEGACY_BOUNDS_VELOCITY_VELOCITYLIMITS_H_
#define _LEGACY_BOUNDS_VELOCITY_VELOCITYLIMITS_H_

 #include <OpenSoT/constraints/velocity/VelocityLimits.h>

 #include <yarp/sig/all.h>

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            /**
             * @brief The VelocityLimits class implements a bound on joint velocities
             */
            class VelocityLimits: public OpenSoT::constraints::velocity::VelocityLimits {
            public:
                VelocityLimits(const double qDotLimit,
                               const double dT,
                               const unsigned int x_size):
                    OpenSoT::constraints::velocity::VelocityLimits(qDotLimit, dT, x_size){}
            };
        }
        }
    }
 }

#endif
