#ifndef _WB_SOT_SOLVERS_BE_FACTORY_H_
#define _WB_SOT_SOLVERS_BE_FACTORY_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/make_shared.hpp>

namespace OpenSoT{
    namespace solvers{
        enum class solver_back_ends{
            qpOASES,
            OSQP
        };

        /**
         * @brief BackEndFactory creates an instance of an OpenSoT BackEnd
         * @param be_solver the type of solver
         * @param number_of_variables of the problem
         * @param number_of_constraints of the problem
         * @param hessian_type of the problem
         * @param eps_regularisation of the problem
         * @return a BackEnd pointer
         */
        BackEnd::Ptr BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type,
                               const double eps_regularisation);

        /**
         * @brief whichBackEnd return a string whith the used BackEnd
         * @param be_solver the input enum
         * @return string
         */
        std::string whichBackEnd(const solver_back_ends be_solver);
    }
}

#endif
