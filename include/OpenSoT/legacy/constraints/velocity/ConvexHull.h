#ifndef _LEGACY_BOUNDS_VELOCITY_CONVEXHULL_H_
#define _LEGACY_BOUNDS_VELOCITY_CONVEXHULL_H_

 #include <OpenSoT/constraints/velocity/ConvexHull.h>

 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {
            class ConvexHull: public OpenSoT::constraints::velocity::ConvexHull {
            public:
                ConvexHull( const yarp::sig::Vector& x,
                            iDynUtils& robot,
                            const double safetyMargin = BOUND_SCALING):
                    OpenSoT::constraints::velocity::ConvexHull(cartesian_utils::toEigen(x),
                        robot, safetyMargin)
                {

                }

                void update(const yarp::sig::Vector& x)
                {
                    OpenSoT::constraints::velocity::ConvexHull::update(cartesian_utils::toEigen(x));
                }

                static void getConstraints(const std::vector<KDL::Vector> &points,
                                            yarp::sig::Matrix& A, yarp::sig::Vector& b,
                                            const double boundScaling = BOUND_SCALING)
                {
                    Eigen::MatrixXd tmpA;
                    Eigen::VectorXd tmpb;
                    OpenSoT::constraints::velocity::ConvexHull::getConstraints(points,
                                            tmpA, tmpb, boundScaling);
                    A = cartesian_utils::fromEigentoYarp(tmpA);
                    b = cartesian_utils::fromEigentoYarp(tmpb);
                }

            };
        }
    }
    }
 }

#endif
