#include <OpenSoT/solvers/BackEndFactory.h>

OpenSoT::solvers::BackEnd::Ptr OpenSoT::solvers::BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type,
                       const double eps_regularisation)
{
    if(be_solver == solver_back_ends::qpOASES)
        return boost::make_shared<QPOasesBackEnd>(
                    QPOasesBackEnd(number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    else
        throw std::runtime_error("Back-end is not available!");

}

std::string OpenSoT::solvers::whichBackEnd(const solver_back_ends be_solver)
{
    if(be_solver == solver_back_ends::qpOASES)
        return "qpOASES";
    else
        return "????";
}


