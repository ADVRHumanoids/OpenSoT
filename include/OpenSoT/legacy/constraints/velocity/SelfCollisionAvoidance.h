#ifndef _LEGACY_SELFCOLLISIONAVOIDANCE_H
#define _LEGACY_SELFCOLLISIONAVOIDANCE_H


 #include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/collision_utils.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
 namespace legacy{
    namespace constraints {
        namespace velocity {

            /**
             * @brief The SelfCollisionAvoidance class implements a constraint of full-body Self-Collision Avoidance for Walkman
             *  This constraint is implemented by inequality: Aineq * x <= bUpperBound
             *  where the dimension of Aineq is n * m, n is the number of Link pairs to be constrained, and m is total DOFs of the robot to be controlled;
             *  the x is infinitesimal increament of the joint variable vector which is the optimization variable, and its dimension is m * 1; 
             *  the bUpperBound is the minimum distance vector of all the Link pairs, the dimension of which is n * 1.
             *  the element in bUpperBound is the minimum distance between the corresponding Link pair with taking the Link pair threshold into account.
             */
            class SelfCollisionAvoidance: public OpenSoT::constraints::velocity::SelfCollisionAvoidance {
            public:
                public:
                SelfCollisionAvoidance(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       double detection_threshold = std::numeric_limits<double>::infinity(),
                                       double linkPair_threshold = 0.0,
                                       const double boundScaling = 1.0):
                    OpenSoT::constraints::velocity::SelfCollisionAvoidance(
                        cartesian_utils::toEigen(x),
                        robot,
                        detection_threshold,
                        linkPair_threshold,
                        boundScaling)
                {

                }

                void update(const yarp::sig::Vector &x)
                {
                    OpenSoT::constraints::velocity::SelfCollisionAvoidance::update(
                                cartesian_utils::toEigen(x));
                }



            };
        }
        }
    }
 }


#endif

