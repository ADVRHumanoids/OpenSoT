#include <OpenSoT/solvers/BackEndFactory.h>
#include <XBotInterface/SoLib.h>

OpenSoT::solvers::BackEnd::Ptr OpenSoT::solvers::BackEndFactory(const solver_back_ends be_solver, const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type,
                       const double eps_regularisation)
{
    if(be_solver == solver_back_ends::qpOASES) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndQPOases.so",
                                          "OpenSotBackEndQPOases",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::OSQP) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndOSQP.so",
                                          "OpenSotBackEndOSQP",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::GLPK) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndGLPK.so",
                                          "OpenSotBackEndGLPK",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::eiQuadProg) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndeiQuadProg.so",
                                          "OpenSotBackEndeiQuadProg",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::ODYS) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndODYS.so",
                                          "OpenSotBackEndODYS",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::qpSWIFT) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndqpSWIFT.so",
                                          "libOpenSotBackEndqpSWIFT",
                                          number_of_variables, number_of_constraints, hessian_type, eps_regularisation));
    }

    if(be_solver == solver_back_ends::proxQP) {

        return std::shared_ptr<BackEnd>(SoLib::getFactoryWithArgs<BackEnd>("libOpenSotBackEndproxQP.so",
                                          "libOpenSotBackEndproxQP",
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
    if(be_solver == solver_back_ends::GLPK)
        return "GLPK";
    if(be_solver == solver_back_ends::eiQuadProg)
        return "eiQuadProg";
    if(be_solver == solver_back_ends::ODYS)
        return "ODYS";
    if(be_solver == solver_back_ends::proxQP)
        return "proxQP";
    if(be_solver == solver_back_ends::qpSWIFT)
        return "qpSWIFT";
    else
        return "????";
}


