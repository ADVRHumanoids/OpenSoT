#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <ctime>
#include <qpOASES/Utils.hpp>
#include <fstream>
#include <boost/make_shared.hpp>
#include <iostream>
#include <qpOASES/Matrices.hpp>
#include <XBotInterface/Logger.hpp>
#include <XBotInterface/SoLib.h>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;


/* Define factories for dynamic loading */
extern "C" BackEnd * create_instance(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation)
{
    return new QPOasesBackEnd(number_of_variables, number_of_constraints, hessian_type, eps_regularisation);
}

extern "C" void destroy_instance( BackEnd * instance )
{
    delete instance;
}

QPOasesBackEnd::QPOasesBackEnd(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation):
    BackEnd(number_of_variables, number_of_constraints),
    _problem(new qpOASES::SQProblem(number_of_variables,
                                    number_of_constraints,
                                    (qpOASES::HessianType)(hessian_type))),
    _bounds(new qpOASES::Bounds()),
    _constraints(new qpOASES::Constraints()),
    _nWSR(13200),
    _epsRegularisation(eps_regularisation),
    _dual_solution(number_of_variables),
    _opt(new qpOASES::Options())
{
    setDefaultOptions();
}

QPOasesBackEnd::~QPOasesBackEnd()
{}

void QPOasesBackEnd::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToMPC();
    opt.printLevel = qpOASES::PL_NONE;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= _epsRegularisation;
    opt.numRegularisationSteps = 0;
    opt.numRefinementSteps = 1;
//    opt.enableFlippingBounds = qpOASES::BT_TRUE; // <- THIS IS NOT RT SAFE!
//     opt.enableDropInfeasibles = qpOASES::BT_TRUE;

    opt.ensureConsistency();

    _problem->setOptions(opt);

    XBot::Logger::info("Solver Default Options: \n");
    opt.print();

    _opt.reset();
    _opt.reset(new qpOASES::Options(opt));
}

void QPOasesBackEnd::setOptions(const boost::any &options){
    _opt.reset();
    _opt.reset(new qpOASES::Options(boost::any_cast<qpOASES::Options>(options)));
    _problem->setOptions(boost::any_cast<qpOASES::Options>(options));}

boost::any QPOasesBackEnd::getOptions(){
    return _problem->getOptions();}

bool QPOasesBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                 const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
    checkINFTY();


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

    int nWSR = _nWSR;

    /**
     * this typedef is needed since qpOASES wants RoWMajor organization
     * of matrices. Thanks to Arturo Laurenzi for the help finding this issue!
     */
    qpOASES::returnValue val =_problem->init(_H.data(),_g.data(),
                       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(_A).data(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(qpOASES::getSimpleStatus(val) < 0)
    {
#ifdef OPENSOT_VERBOSE
        _problem->printProperties();

        if(val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
            checkInfeasibility();

        XBot::Logger::error("ERROR INITIALIZING QP PROBLEM \n");
        XBot::Logger::error("CODE ERROR: %i \n", val);
#endif

        return false;
    }

    if(_solution.rows() != _problem->getNV())
        _solution.resize(_problem->getNV());

    if(_dual_solution.rows() != _problem->getNV() + _problem->getNC())
        _dual_solution.resize(_problem->getNV() + _problem->getNC());

    //We get the solution
    qpOASES::returnValue success = _problem->getPrimalSolution(_solution.data());
    _problem->getDualSolution(_dual_solution.data());
    _problem->getBounds(*_bounds);
    _problem->getConstraints(*_constraints);

    if(success != qpOASES::SUCCESSFUL_RETURN){
#ifdef OPENSOT_VERBOSE
        XBot::Logger::error("ERROR GETTING PRIMAL SOLUTION IN INITIALIZATION! ERROR %i \n", success);
#endif
        return false;}
    return true;
}

bool QPOasesBackEnd::updateTask(const Eigen::MatrixXd &H, const Eigen::VectorXd &g)
{
    if(!(_g.rows() == _H.rows())){
        XBot::Logger::error("g size: %i \n", _g.rows());
        XBot::Logger::error("H size: %i \n", _H.rows());
        assert(_g.rows() == _H.rows());
        return false;}
    if(!(_H.cols() == H.cols())){
        std::cout<<RED<<"H cols: "<<H.cols()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
        return false;}

    if(_H.rows() == H.rows())
    {
        _H = H;
        _g = g;

        return true;
    }
    else
    {
        _H = H;
        _g = g;

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _H.cols();
        int number_of_constraints = _A.rows();
        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        _problem->setOptions(*_opt.get());
        return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
    }
}

bool QPOasesBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A,
                               const Eigen::Ref<const Eigen::VectorXd> &lA, 
                               const Eigen::Ref<const Eigen::VectorXd> &uA)
{
    if(!(A.cols() == _H.cols())){
        std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
        return false;}
    if(!(lA.rows() == A.rows())){
        std::cout<<RED<<"lA size: "<<lA.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"A rows: "<<A.rows()<<DEFAULT<<std::endl;
        return false;}
    if(!(lA.rows() == uA.rows())){
        std::cout<<RED<<"lA size: "<<lA.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"uA size: "<<uA.rows()<<DEFAULT<<std::endl;
        return false;}

    if(A.rows() == _A.rows())
    {
        _A = A;
        _lA = lA;
        _uA = uA;
        return true;
    }
    else
    {
        _A = A;
        _lA = lA;
        _uA = uA;

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _H.cols();
        int number_of_constraints = _A.rows();
        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        _problem->setOptions(*_opt.get());
        return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
    }
}


bool QPOasesBackEnd::solve()
{
    int nWSR = _nWSR;
    checkINFTY();

    qpOASES::returnValue val =_problem->hotstart(_H.data(),_g.data(),
                       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(_A).data(),
                        _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(val != qpOASES::SUCCESSFUL_RETURN){
#ifdef OPENSOT_VERBOSE
        std::cout<<YELLOW<<"WARNING OPTIMIZING TASK IN HOTSTART! ERROR "<<val<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"RETRYING INITING WITH WARMSTART"<<DEFAULT<<std::endl;
#endif

        val =_problem->init(_H.data(),_g.data(),
                           Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(_A).data(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           nWSR,0,
                           _solution.data(), _dual_solution.data(),
                           _bounds.get(), _constraints.get());

        if(val != qpOASES::SUCCESSFUL_RETURN){
#ifdef OPENSOT_VERBOSE
            std::cout<<YELLOW<<"WARNING OPTIMIZING TASK IN WARMSTART! ERROR "<<val<<DEFAULT<<std::endl;
            std::cout<<GREEN<<"RETRYING INITING"<<DEFAULT<<std::endl;
#endif

            return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);}
    }

    // If solution has changed of size we update the size
    if(_solution.rows() != _problem->getNV())
        _solution.resize(_problem->getNV());

    if(_dual_solution.rows() != _problem->getNV() + _problem->getNC())
        _dual_solution.resize(_problem->getNV()+ _problem->getNC());

    //We get the solution
    qpOASES::returnValue success = _problem->getPrimalSolution(_solution.data());
    _problem->getDualSolution(_dual_solution.data());
    _problem->getBounds(*_bounds);
    _problem->getConstraints(*_constraints);

    if(qpOASES::getSimpleStatus(success) < 0){
#ifdef OPENSOT_VERBOSE
        std::cout<<"ERROR GETTING PRIMAL SOLUTION! ERROR "<<success<<std::endl;
#endif
        return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);
    }
    return true;
}


OpenSoT::HessianType QPOasesBackEnd::getHessianType() {return (OpenSoT::HessianType)(_problem->getHessianType());}

void QPOasesBackEnd::setHessianType(const OpenSoT::HessianType ht){_problem->setHessianType((qpOASES::HessianType)(ht));}

void QPOasesBackEnd::checkInfeasibility()
{
    qpOASES::Constraints infeasibleConstraints;
    _problem->getConstraints(infeasibleConstraints);
    std::cout<<RED<<"Constraints:"<<DEFAULT<<std::endl;
    infeasibleConstraints.print();

    std::cout<<"--------------------------------------------"<<std::endl;
    for(unsigned int i = 0; i < _lA.rows(); ++i)
        std::cout<<i<<": "<<_lA[i]<<" <= "<<"Adq"<<" <= "<<_uA[i]<<std::endl;

    std::cout<<std::endl;
    std::cout<<"A = ["<<std::endl;
    std::cout<<_A<<" ]"<<std::endl;
    std::cout<<"--------------------------------------------"<<std::endl;
}

void QPOasesBackEnd::_printProblemInformation()
{
    std::ostringstream oss;
    oss << std::scientific << _problem->getOptions().epsRegularisation;
    XBot::Logger::info("eps Regularisation factor: %s \n", oss.str().c_str());
    XBot::Logger::info("qpOASES # OF CONSTRAINTS: %i\n", _problem->getNC());
    XBot::Logger::info("qpOASES # OF VARIABLES: %i\n", _problem->getNV());
}

double QPOasesBackEnd::getObjective()
{
    return _problem->getObjVal();
}

void QPOasesBackEnd::checkINFTY()
{
    unsigned int constraints_size = _lA.rows();
    for(unsigned int i = 0; i < constraints_size; ++i){
        if(_lA[i] < -qpOASES::INFTY)
            _lA[i] = -qpOASES::INFTY;
        if(_uA[i] > qpOASES::INFTY)
            _uA[i] = qpOASES::INFTY;}

    unsigned int bounds_size = _l.rows();
    for(unsigned int i = 0; i < bounds_size; ++i){
        if(_l[i] < -qpOASES::INFTY)
            _l[i] = -qpOASES::INFTY;
        if(_u[i] > qpOASES::INFTY)
            _u[i] = qpOASES::INFTY;}
}
