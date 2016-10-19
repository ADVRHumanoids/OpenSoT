#ifndef _LEGACY_WB_SOT_SOLVERS_QP_OASES_PROBLEM_H_
#define _LEGACY_WB_SOT_SOLVERS_QP_OASES_PROBLEM_H_

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <idynutils/cartesian_utils.h>
#include <OpenSoT/solvers/QPOasesProblem.h>

using namespace yarp::sig;

namespace OpenSoT{
namespace legacy{
    namespace solvers{

    class QPOasesProblem: public OpenSoT::solvers::QPOasesProblem {
    public:
    QPOasesProblem(const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type = OpenSoT::HST_UNKNOWN,
                       const double eps_regularisation = DEFAULT_EPS_REGULARISATION):
        OpenSoT::solvers::QPOasesProblem(number_of_variables, number_of_constraints,
                                         hessian_type, eps_regularisation)
    {}


        bool initProblem(const yarp::sig::Matrix& H, const yarp::sig::Vector& g,
                        const yarp::sig::Matrix& A,
                        const yarp::sig::Vector& lA, const yarp::sig::Vector& uA,
                        const yarp::sig::Vector& l, const yarp::sig::Vector& u)
        {
            return OpenSoT::solvers::QPOasesProblem::initProblem(cartesian_utils::toEigen(H), cartesian_utils::toEigen(g),
                                                          cartesian_utils::toEigen(A),
                                                          cartesian_utils::toEigen(lA), cartesian_utils::toEigen(uA),
                                                          cartesian_utils::toEigen(l), cartesian_utils::toEigen(u));
        }

        bool updateTask(const yarp::sig::Matrix& H, const yarp::sig::Vector& g)
        {
            return OpenSoT::solvers::QPOasesProblem::updateTask(cartesian_utils::toEigen(H), cartesian_utils::toEigen(g));
        }

        bool updateConstraints(const yarp::sig::Matrix& A, const yarp::sig::Vector& lA, const yarp::sig::Vector& uA)
        {
            return OpenSoT::solvers::QPOasesProblem::updateConstraints(cartesian_utils::toEigen(A),
                                                    cartesian_utils::toEigen(lA), cartesian_utils::toEigen(uA));
        }

        bool updateBounds(const yarp::sig::Vector& l, const yarp::sig::Vector& u)
        {
            return OpenSoT::solvers::QPOasesProblem::updateBounds(cartesian_utils::toEigen(l), cartesian_utils::toEigen(u));
        }

        bool updateProblem(const yarp::sig::Matrix& H, const yarp::sig::Vector& g,
                           const yarp::sig::Matrix& A,
                           const yarp::sig::Vector& lA, const yarp::sig::Vector& uA,
                           const yarp::sig::Vector& l, const yarp::sig::Vector& u)
        {
            return OpenSoT::solvers::QPOasesProblem::updateProblem(cartesian_utils::toEigen(H), cartesian_utils::toEigen(g),
                            cartesian_utils::toEigen(A), cartesian_utils::toEigen(lA), cartesian_utils::toEigen(uA), cartesian_utils::toEigen(l), cartesian_utils::toEigen(u));
        }

        bool addTask(const yarp::sig::Matrix& H, const yarp::sig::Vector& g)
        {
            return OpenSoT::solvers::QPOasesProblem::addTask(cartesian_utils::toEigen(H), cartesian_utils::toEigen(g));
        }

        bool addConstraints(const yarp::sig::Matrix& A, const yarp::sig::Vector& lA, const yarp::sig::Vector& uA)
        {
            return OpenSoT::solvers::QPOasesProblem::addConstraints(cartesian_utils::toEigen(A),
                                                            cartesian_utils::toEigen(lA), cartesian_utils::toEigen(uA));
        }
    };
    }
}
}
#endif
