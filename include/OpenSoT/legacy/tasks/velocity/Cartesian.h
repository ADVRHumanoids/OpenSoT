#ifndef _LEGACY_TASKS_VELOCITY_CARTESIAN_H_
#define _LEGACY_TASKS_VELOCITY_CARTESIAN_H_

#include <OpenSoT/tasks/velocity/Cartesian.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>

 #define WORLD_FRAME_NAME "world"

/**
 * @example example_cartesian.cpp
 * The Cartesian class implements a task that tries to impose a pose (position and orientation)
 * of a distal link w.r.t. a base link.
 */

 namespace OpenSoT {
 namespace legacy{
    namespace tasks {
        namespace velocity {
            /**
             * @brief The Cartesian class implements a task that tries to impose a pose (position and orientation)
             * of a distal link w.r.t. a base link. The reference for the cartesian task is set in base link
             * coordinate frame, or in world if the base link name is set to "world".
             * The Cartesian Task is implemented so that
             * \f$A={}^\text{base}J_\text{distal}\f$
             * and
             * \f$b=K_p*e+\frac{1}{\lambda}\xi_d\f$
             *
             * You can see an example in @ref example_cartesian.cpp
             */
            class Cartesian : public OpenSoT::tasks::velocity::Cartesian {
            public:

                Cartesian(std::string task_id,
                          const yarp::sig::Vector& x,
                          iDynUtils &robot,
                          std::string distal_link,
                          std::string base_link):
                    OpenSoT::tasks::velocity::Cartesian(task_id,
                        cartesian_utils::toEigen(x), robot,
                        distal_link, base_link)
                {}


                void setReference(const yarp::sig::Matrix& desiredPose)
                {
                    OpenSoT::tasks::velocity::Cartesian::setReference(cartesian_utils::toEigen(desiredPose));
                }

                void setReference(const yarp::sig::Matrix& desiredPose,
                                  const yarp::sig::Vector& desiredTwist)
                {
                    OpenSoT::tasks::velocity::Cartesian::setReference(
                                cartesian_utils::toEigen(desiredPose),
                                cartesian_utils::toEigen(desiredTwist));
                }

                const yarp::sig::Matrix getReference()
                {
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::tasks::velocity::Cartesian::getReference());
                }

                void getReference(yarp::sig::Matrix& desiredPose,
                                  yarp::sig::Vector& desiredTwist)
                {
                    Eigen::MatrixXd tmp;
                    Eigen::VectorXd tmp2;
                    OpenSoT::tasks::velocity::Cartesian::getReference(tmp, tmp2);

                    desiredPose = cartesian_utils::fromEigentoYarp(tmp);
                    desiredTwist = cartesian_utils::fromEigentoYarp(tmp2);
                }


                const yarp::sig::Matrix getActualPose()
                {
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::tasks::velocity::Cartesian::getActualPose());
                }
                
                const KDL::Frame getActualPoseKDL()
                {
                    return OpenSoT::tasks::velocity::Cartesian::getActualPoseKDL();
                }


                yarp::sig::Vector getError()
                {
                    return cartesian_utils::fromEigentoYarp(
                                OpenSoT::tasks::velocity::Cartesian::getError());
                }
                


            };
        }
        }
    }
 }

#endif
