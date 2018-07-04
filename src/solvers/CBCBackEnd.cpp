#include <OpenSoT/solvers/CBCBackEnd.h>
#include <XBotInterface/SoLib.h>
#include <boost/make_shared.hpp>

using namespace OpenSoT::solvers;

/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables, const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new CBCBackEnd(number_of_variables, number_of_constraints, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

CBCBackEnd::CBCBackEnd(const int number_of_variables, const int number_of_constraints, const double eps_regularisation): 
    BackEnd(number_of_variables, number_of_constraints)
{   
    __generate_data_struct(number_of_variables, number_of_constraints);
    _integer_variables.reserve(number_of_variables);
}

bool CBCBackEnd::solve()
{
    _model->resetToReferenceSolver();
    
    _model->solver()->loadProblem(_ACP, _l.data(), _u.data(), _g.data(), _lA.data(), _uA.data());

     for(unsigned int i = 0; i < _integer_variables.size(); ++i)
         _model->solver()->setInteger(_integer_variables[i]);
     
    _model->solver()->initialSolve();
     
    _model->solver()->branchAndBound();
    
    
    if(!_model->isProvenInfeasible())
        _solution = Eigen::Map<const Eigen::VectorXd>(_model->solver()->getColSolution(), getNumVariables());
    else
    {
        XBot::Logger::error("CbcModel return unfeasible solution in solve!");
        return false;
    }
    return true;
}


bool CBCBackEnd::initProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, 
                             const Eigen::MatrixXd& A, const Eigen::VectorXd& lA, const Eigen::VectorXd& uA, 
                             const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
    
    if(!(_l.rows() == _u.rows())){
        XBot::Logger::error("l size: %i \n", _l.rows());
        XBot::Logger::error("u size: %i \n", _u.rows());
        assert(_l.rows() == _u.rows());
        return false;}
    if(!(_lA.rows() == _A.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("A rows: %i \n", _A.rows());
        assert(_lA.rows() == _A.rows());
        return false;}
    if(!(_lA.rows() == _uA.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("uA size: %i \n", _uA.rows());
        assert(_lA.rows() == _uA.rows());
        return false;}
        
    _ACP.copyOf(true, _A.rows(), _A.cols(), _AS.nonZeros(), _A.data(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());

    OsiCbcSolverInterface *_solver = new OsiCbcSolverInterface;

    _solver->loadProblem(_ACP, _l.data(), _u.data(), _g.data(), _lA.data(), _uA.data());

    for(unsigned int i = 0; i < _integer_variables.size(); ++i)
        _solver->setInteger(_integer_variables[i]);

    _solver->initialSolve();

    _model.reset(new CbcModel(*_solver));
    _model->setLogLevel(0);
    _model->solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);
    _model->branchAndBound(); 

    
    if(!_model->isProvenInfeasible())
        _solution = Eigen::Map<const Eigen::VectorXd>(_model->solver()->getColSolution(), getNumVariables());

    else
    {
        XBot::Logger::error("CbcModel return unfeasible solution in initProblem!");
        return false;
    }
    return true;
}

boost::any CBCBackEnd::getOptions()
{
    return _opt;
}


void CBCBackEnd::setOptions(const boost::any& options)
{
    _opt = boost::any_cast<CBCBackEndOptions>(options);
    

    auto it_wrong = std::find_if(_opt.integer_ind.begin(), _opt.integer_ind.end(), [this](int i){ return i >= getNumVariables(); });
    if(it_wrong != _opt.integer_ind.end())
        XBot::Logger::error("Index of integer variable greater than number of variables! Options will not be applied!");
    else
    {
        if(!_integer_variables.empty())
            _integer_variables.clear();

        if(_opt.integer_ind.size() <= getNumVariables())
        {
             for(unsigned int i = 0; i < _opt.integer_ind.size(); ++i)
                 _integer_variables.push_back(_opt.integer_ind[i]);
        }
        else
             XBot::Logger::error("Size of integer variable greater than number of variables! Options will not be applied!");
    }

    __H = _H; __g = _g; __A = _A; __lA = _lA; __uA = _uA; __l = _l; __u = _u;
    initProblem(__H, __g, __A, __lA, __uA, __l, __u);
}

void CBCBackEnd::__generate_data_struct(const int number_of_variables, 
                                        const int number_of_constraints)
{
    
    /* Set appropriate sparsity pattern to P (upper triangular) */
    
    Eigen::MatrixXd sp_pattern;
    /* Set appropriate sparsity pattern to A (dense + diagonal) */
    sp_pattern.setOnes(number_of_constraints, number_of_variables);
    
    _AS = sp_pattern.sparseView();
    _AS.makeCompressed();

   
    _ACP.copyOf(true, _AS.rows(), _AS.cols(), _AS.nonZeros(), _AS.valuePtr(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());
}

bool CBCBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A, 
                                const Eigen::Ref<const Eigen::VectorXd>& lA, 
                                const Eigen::Ref<const Eigen::VectorXd>& uA)
{    
    if(A.rows())
    {
        bool success = BackEnd::updateConstraints(A, lA, uA);

        if(!success)
        {
            return false;
        }
        

        _ACP.copyOf(true, _A.rows(), _A.cols(), _AS.nonZeros(), _A.data(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());
        
    }
    
    return true;
}

CBCBackEnd::~CBCBackEnd()
{

}

double CBCBackEnd::getObjective()
{
   return _model->getObjValue();
}

void CBCBackEnd::_printProblemInformation()
{
    XBot::Logger::info("CBC # OF INTEGER VARIABLES: %i \n", _model->numberIntegers());
    XBot::Logger::info("CBC INTEGER VARIABLES: [ ");
    for(unsigned int i = 0; i < _model->numberIntegers(); ++i)
        XBot::Logger::log()<<_model->integerVariable()[i]<<" ";
    XBot::Logger::log()<<"]"<<XBot::Logger::endl();

}
    
