#include <OpenSoT/solvers/BackEndFactory.h>
#include <XBotInterface/SoLib.h>
#include <boost/bind.hpp>

/* Utility to convert from std to boost shared pointer */
namespace {
    template<typename T>
    void do_release(typename std::shared_ptr<T> const&, T*)
    {
    }

    template<typename T>
    typename boost::shared_ptr<T> to_boost(typename std::shared_ptr<T> const& p)
    {
        return
            boost::shared_ptr<T>(
                    p.get(),
                    boost::bind(&do_release<T>, p, _1));

    }
}


OpenSoT::solvers::BackEnd::Ptr OpenSoT::solvers::BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type,
                       const double eps_regularisation)
{
    if(be_solver == solver_back_ends::qpOASES)
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("OpenSotBackEndQPOases.so", 
                                                  "OpenSotBackEndQPOases", 
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    if(be_solver == solver_back_ends::OSQP)
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("OpenSotBackEndOSQP.so", 
                                                  "OpenSotBackEndOSQP", 
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    else
        throw std::runtime_error("Back-end is not available!");

}

std::string OpenSoT::solvers::whichBackEnd(const solver_back_ends be_solver)
{
    if(be_solver == solver_back_ends::qpOASES)
        return "qpOASES";
    if(be_solver == solver_back_ends::OSQP)
        return "OSQP";
    else
        return "????";
}


