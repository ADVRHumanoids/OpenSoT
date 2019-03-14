#include <OpenSoT/solvers/uQuadProgBackEnd.h>

using namespace OpenSoT::solvers;

#define BASE_REGULARISATION 1E-12

/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new uQuadProgBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

uQuadProgBackEnd::uQuadProgBackEnd(const int number_of_variables,
                                   const int number_of_constraints,
                                   const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _eps_regularisation(eps_regularisation),
    /** initialization of uQuadProg matrices and vectors **/
    _G(number_of_variables, number_of_variables),
    _g0(number_of_variables),
    _CE(0,0),
    _ce0(0),
    _CI(2*number_of_constraints,number_of_variables),
    _ci0(2*number_of_constraints),
    /** initialization of Pilers **/
    _CIPiler(number_of_variables),
    _ci0Piler(number_of_variables),
    /** solution **/
    _x(number_of_variables,0)
{
    _I.setIdentity(number_of_variables, number_of_variables);

}

void uQuadProgBackEnd::__generate_data_struct()
{
    std::cout<<"_l: "<<_l.transpose()<<std::endl;
    std::cout<<"_u: "<<_u.transpose()<<std::endl;


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

        /** TODO: use eigen maps instead! **/
        for(unsigned int i = 0; i < _CIPiler.generate_and_get().rows(); ++i)
        {
            _ci0(i) = _ci0Piler.generate_and_get()(i,0);
            for(unsigned int j = 0; j < _CIPiler.generate_and_get().cols(); ++j)
                _CI(i,j) = _CIPiler.generate_and_get()(i,j);
        }

    //Cost Function (here we add manual regularisation)
        /** TODO: use eigen maps instead! **/
        for(unsigned int i = 0; i < _g.size(); ++i)
        {
            _g0(i) = _g(i);
            for(unsigned int j = 0; j < _g.size(); ++j)
                _G(i,j) = _H(i,j);
        }
        for(int i = 0; i < getNumVariables(); ++i)
            _G(i,i) += _eps_regularisation*BASE_REGULARISATION; //TO HAVE COMPATIBILITY WITH THE QPOASES ONE!
                                                                // Not sure if should be _eps_regularisation**2!
}

bool uQuadProgBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                         const Eigen::MatrixXd &A,
                         const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                         const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u; //this is needed since updateX should be used just to update and not init (maybe can be done in the base class)
    __generate_data_struct();

    bool success = true;
    const double inf = std::numeric_limits<double>::infinity();

    _f_value = solve_quadprog(_G, _g0, _CE, _ce0, _CI, _ci0, _x);
    if(_f_value == inf)
        return false;
    for(unsigned int i = 0; i < _x.size(); ++i)
        _solution(i) = _x(i);
}

bool uQuadProgBackEnd::solve()
{
    __generate_data_struct();

    bool success = true;
    const double inf = std::numeric_limits<double>::infinity();

    _f_value = solve_quadprog(_G, _g0, _CE, _ce0, _CI, _ci0, _x);
    if(_f_value == inf)
        return false;
    for(unsigned int i = 0; i < _x.size(); ++i)
        _solution(i) = _x(i);
}

double uQuadProgBackEnd::getObjective()
{
    return _f_value;
}

boost::any uQuadProgBackEnd::getOptions()
{
    ///DO NOTHING
}

void uQuadProgBackEnd::setOptions(const boost::any& options)
{
    ///DO NOTHING
}



