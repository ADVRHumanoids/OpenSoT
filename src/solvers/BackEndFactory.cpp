#include <OpenSoT/solvers/BackEndFactory.h>
#include <xbot2_interface/common/dynamic_loading.h>

OpenSoT::solvers::BackEnd::Ptr CreateBackend(std::string name,
                                             const int number_of_variables,
                                             const int number_of_constraints,
                                             OpenSoT::HessianType hessian_type,
                                             const double eps_regularisation)
{
    return OpenSoT::solvers::BackEnd::Ptr(
        XBot::Utils::CallFunction<OpenSoT::solvers::BackEnd *>("libOpenSotBackEnd" + name + ".so",
                                                               "create_instance",
                                                               number_of_variables,
                                                               number_of_constraints,
                                                               hessian_type,
                                                               eps_regularisation));
}

OpenSoT::solvers::BackEnd::Ptr OpenSoT::solvers::BackEndFactory(const solver_back_ends be_solver,
                                                                const int number_of_variables,
                                                                const int number_of_constraints,
                                                                OpenSoT::HessianType hessian_type,
                                                                const double eps_regularisation)
{
    if (be_solver == solver_back_ends::qpOASES) {
        return CreateBackend("QPOases",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::OSQP) {
        return CreateBackend("OSQP",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::GLPK) {
        return CreateBackend("GLPK",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::eiQuadProg) {
        return CreateBackend("eiQuadProg",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::ODYS) {
        return CreateBackend("ODYS",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::qpSWIFT) {
        return CreateBackend("qpSWIFT",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    if (be_solver == solver_back_ends::proxQP) {
        return CreateBackend("proxQP",
                             number_of_variables,
                             number_of_constraints,
                             hessian_type,
                             eps_regularisation);
    }

    else {
        throw std::runtime_error("Back-end is not available!");
    }
}

std::string OpenSoT::solvers::whichBackEnd(const solver_back_ends be_solver)
{
    if (be_solver == solver_back_ends::qpOASES)
        return "qpOASES";
    if (be_solver == solver_back_ends::OSQP)
        return "OSQP";
    if (be_solver == solver_back_ends::GLPK)
        return "GLPK";
    if (be_solver == solver_back_ends::eiQuadProg)
        return "eiQuadProg";
    if (be_solver == solver_back_ends::ODYS)
        return "ODYS";
    if (be_solver == solver_back_ends::proxQP)
        return "proxQP";
    if (be_solver == solver_back_ends::qpSWIFT)
        return "qpSWIFT";
    else
        return "????";
}
