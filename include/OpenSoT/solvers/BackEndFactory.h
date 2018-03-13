#ifndef _WB_SOT_SOLVERS_BE_FACTORY_H_
#define _WB_SOT_SOLVERS_BE_FACTORY_H_

#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <boost/make_shared.hpp>

namespace OpenSoT{
    namespace solvers{
        enum class solver_back_ends{
            qpOASES,
            OSQP
        };

        BackEnd::Ptr BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type = OpenSoT::HST_UNKNOWN,
                               const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        std::string whichBackEnd(const solver_back_ends be_solver);
    }
}

#endif
