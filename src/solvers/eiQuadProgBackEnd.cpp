#include <OpenSoT/solvers/eiQuadProgBackEnd.h>
#include <XBotInterface/Logger.hpp>


using namespace OpenSoT::solvers;

#define BASE_REGULARISATION 1E-12

/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new eiQuadProgBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

eiQuadProgBackEnd::eiQuadProgBackEnd(const int number_of_variables,
                                   const int number_of_constraints,
                                   const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _eps_regularisation(eps_regularisation),
    /** initialization of Pilers **/
    _CIPiler(number_of_variables),
    _ci0Piler(1)
{
    _I.setIdentity(number_of_variables, number_of_variables);
}


void eiQuadProgBackEnd::__generate_data_struct()
{
    //Bounds & Constraints
        _CIPiler.reset();
        _ci0Piler.reset();

        //1) Bounds
        if(_l.size() > 0)
        {
            _CIPiler.pile(_I);
            _ci0Piler.pile(-_l);
            _CIPiler.pile(-_I);
            _ci0Piler.pile(_u);
        }

        //2) Constraints
        if(_A.rows() > 0)
        {
            _CIPiler.pile(_A);
            _ci0Piler.pile(-_lA);
            _CIPiler.pile(-_A);
            _ci0Piler.pile(_uA);
        }

        for(int i = 0; i < _H.rows(); ++i)
            _H(i,i) += _eps_regularisation*BASE_REGULARISATION;
}

bool eiQuadProgBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                         const Eigen::MatrixXd &A,
                         const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                         const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    //couple of checks
    if(A.rows() != _A.rows()){
        XBot::Logger::error("A.rows() != _A.rows() --> %f != %f", A.rows(), _A.rows());
        return false;}

    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u; //this is needed since updateX should be used just to update and not init (maybe can be done in the base class)
    __generate_data_struct();

    const double inf = std::numeric_limits<double>::infinity();


    _f_value = solve_quadprog(_H, _g, Eigen::MatrixXd(), Eigen::VectorXd(),
                              _CIPiler.generate_and_get().transpose(),
                              _ci0Piler.generate_and_get(),
                              _solution);
    if(_f_value == inf)
    {
        XBot::Logger::error("QPP is infeasible");
        return false;
    }
    return true;

}

bool eiQuadProgBackEnd::solve()
{
    __generate_data_struct();

    const double inf = std::numeric_limits<double>::infinity();


    _f_value = solve_quadprog(_H, _g, Eigen::MatrixXd(), Eigen::VectorXd(),
                              _CIPiler.generate_and_get().transpose(),
                              _ci0Piler.generate_and_get(),
                              _solution);
    if(_f_value == inf)
    {
        XBot::Logger::error("QPP is infeasible");
        return false;
    }
    return true;
}

double eiQuadProgBackEnd::getObjective()
{
    return _f_value;
}

boost::any eiQuadProgBackEnd::getOptions()
{
    ///DO NOTHING
}

void eiQuadProgBackEnd::setOptions(const boost::any& options)
{
    ///DO NOTHING
}



