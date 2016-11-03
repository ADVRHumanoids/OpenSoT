#ifndef _LEGACY_BOUNDS_VELOCITY_CartesianPositionConstraint_H__
#define _LEGACY_BOUNDS_VELOCITY_CartesianPositionConstraint_H__

 #include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
 #include <yarp/sig/all.h>

#define BOUND_SCALING 0.01

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            class CartesianPositionConstraint: public OpenSoT::constraints::velocity::CartesianPositionConstraint {
            public:
                CartesianPositionConstraint(const yarp::sig::Vector& x,
                                             OpenSoT::tasks::velocity::Cartesian::Ptr cartesianTask,
                                             const yarp::sig::Matrix& A_Cartesian,
                                             const yarp::sig::Vector& b_Cartesian,
                                            const double boundScaling = 1.0):
                    OpenSoT::constraints::velocity::CartesianPositionConstraint(
                        cartesian_utils::toEigen(x),
                        cartesianTask,
                        cartesian_utils::toEigen(A_Cartesian),
                        cartesian_utils::toEigen(b_Cartesian),
                        boundScaling)
                {

                }

                void update(const yarp::sig::Vector &x)
                {
                    OpenSoT::constraints::velocity::CartesianPositionConstraint::update(
                                cartesian_utils::toEigen(x));
                }
            };
        }
        }
    }
 }

#endif
