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
    if(be_solver == solver_back_ends::qpOASES) { 

        /* Obtain full path to shared lib */
        std::string path_to_shared_lib = XBot::Utils::FindLib("libOpenSotBackEndQPOases.so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "") {
            throw std::runtime_error("libOpenSotBackEndQPOases.so must be listed inside LD_LIBRARY_PATH");
        }
        
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>(path_to_shared_lib, 
                                                  "OpenSotBackEndQPOases", 
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }
    
    if(be_solver == solver_back_ends::OSQP) { 

        /* Obtain full path to shared lib */
        std::string path_to_shared_lib = XBot::Utils::FindLib("libOpenSotBackEndOSQP.so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "") {
            throw std::runtime_error("libOpenSotBackEndOSQP.so must be listed inside LD_LIBRARY_PATH");
        }
        
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>(path_to_shared_lib, 
                                                  "OpenSotBackEndOSQP", 
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }
    
    if(be_solver == solver_back_ends::CBC) { 

        /* Obtain full path to shared lib */
        std::string path_to_shared_lib = XBot::Utils::FindLib("libOpenSotBackEndCBC.so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "") {
            throw std::runtime_error("libOpenSotBackEndCBC.so must be listed inside LD_LIBRARY_PATH");
        }
        
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>(path_to_shared_lib,
                                                  "OpenSotBackEndCBC",
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }
    
    if(be_solver == solver_back_ends::GLPK) { 

        /* Obtain full path to shared lib */
        std::string path_to_shared_lib = XBot::Utils::FindLib("libOpenSotBackEndGLPK.so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "") {
            throw std::runtime_error("libOpenSotBackEndGLPK.so must be listed inside LD_LIBRARY_PATH");
        }
        
        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>(path_to_shared_lib,
                                                  "OpenSotBackEndGLPK",
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::uQuadProg) {

        /* Obtain full path to shared lib */
        std::string path_to_shared_lib = XBot::Utils::FindLib("libOpenSotBackEnduQuadProg.so", "LD_LIBRARY_PATH");
        if (path_to_shared_lib == "") {
            throw std::runtime_error("libOpenSotBackEnduQuadProg.so must be listed inside LD_LIBRARY_PATH");
        }

        return to_boost<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>(path_to_shared_lib,
                                                  "OpenSotBackEnduQuadProg",
                                                  number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }
    
    else {
        throw std::runtime_error("Back-end is not available!");
    }

}

std::string OpenSoT::solvers::whichBackEnd(const solver_back_ends be_solver)
{
    if(be_solver == solver_back_ends::qpOASES)
        return "qpOASES";
    if(be_solver == solver_back_ends::OSQP)
        return "OSQP";
    if(be_solver == solver_back_ends::CBC)
        return "CBC";
    if(be_solver == solver_back_ends::GLPK)
        return "GLPK";
    else
        return "????";
}


