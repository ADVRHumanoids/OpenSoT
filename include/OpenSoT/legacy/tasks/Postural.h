#ifndef _LEGACY_POSTURAL_
#define _LEGACY_POSTURAL_

#include <OpenSoT/tasks/velocity/Postural.h>
#include <idynutils/cartesian_utils.h>

namespace OpenSoT{
namespace legacy{
namespace tasks{
namespace velocity{

class Postural : public OpenSoT::tasks::velocity::Postural {
            public:

                Postural(const yarp::sig::Vector& x):
                    OpenSoT::tasks::velocity::Postural(cartesian_utils::toEigen(x)) {
                }


                void setReference(const yarp::sig::Vector& x_desired)
                {
                    OpenSoT::tasks::velocity::Postural::setReference(cartesian_utils::toEigen(x_desired));
                }

                void setReference(const yarp::sig::Vector& x_desired,
                                  const yarp::sig::Vector& xdot_desired)
                {
                    OpenSoT::tasks::velocity::Postural::setReference(cartesian_utils::toEigen(x_desired),
                                           cartesian_utils::toEigen(xdot_desired));
                }

                yarp::sig::Vector getReference(){
                    Eigen::VectorXd tmp = OpenSoT::tasks::velocity::Postural::getReference();
                    return cartesian_utils::fromEigentoYarp(tmp);
                }


                void getReference(yarp::sig::Vector& x_desired,
                                  yarp::sig::Vector& xdot_desired)
                {
                    Eigen::VectorXd tmp1;
                    Eigen::VectorXd tmp2;
                    OpenSoT::tasks::velocity::Postural::getReference(tmp1, tmp2);
                    x_desired = cartesian_utils::fromEigentoYarp(tmp1);
                    xdot_desired = cartesian_utils::fromEigentoYarp(tmp2);
                }

                void setLambda(double lambda)
                {
                    OpenSoT::tasks::velocity::Postural::setLambda(lambda);
                }

                yarp::sig::Vector getActualPositions()
                {
                    Eigen::VectorXd tmp = OpenSoT::tasks::velocity::Postural::getActualPositions();
                    return cartesian_utils::fromEigentoYarp(tmp);
                }

                yarp::sig::Vector getError()
                {
                    Eigen::VectorXd tmp = OpenSoT::tasks::velocity::Postural::getError();
                    return cartesian_utils::fromEigentoYarp(tmp);
                }

            };
}
}
}
}

#endif
