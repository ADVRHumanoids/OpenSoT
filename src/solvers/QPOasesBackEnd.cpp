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
    _dual_solution(number_of_variables)
{
    _opt = boost::make_shared<qpOASES::Options>();
#ifdef OPENSOT_VERBOSE
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::LOW);
#endif

    setDefaultOptions();
}

QPOasesBackEnd::~QPOasesBackEnd()
{}

void QPOasesBackEnd::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToMPC();
    opt.printLevel = qpOASES::PL_NONE;
    opt.enableRegularisation = qpOASES::BT_FALSE;
    opt.epsRegularisation *= _epsRegularisation;
    opt.numRegularisationSteps = 0;
    opt.numRefinementSteps = 1;
//    opt.enableFlippingBounds = qpOASES::BT_TRUE; // <- THIS IS NOT RT SAFE!
//     opt.enableDropInfeasibles = qpOASES::BT_TRUE;

    opt.ensureConsistency();

    _problem->setOptions(opt);

    _epsRegularisation = opt.epsRegularisation;
    XBot::Logger::info("Solver Default Options: \n");
    opt.print();

    _opt.reset();
    _opt = boost::make_shared<qpOASES::Options>(opt);
}

void QPOasesBackEnd::setOptions(const boost::any &options){
    _opt.reset();
    _opt = boost::make_shared<qpOASES::Options>(boost::any_cast<qpOASES::Options>(options));
    _problem->setOptions(boost::any_cast<qpOASES::Options>(options));}

boost::any QPOasesBackEnd::getOptions(){
    return _problem->getOptions();}

bool QPOasesBackEnd::initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                 const Eigen::MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    //couple of checks
    if(A.rows() != _A.rows()){
        XBot::Logger::error("A.rows() != _A.rows() --> %f != %f", A.rows(), _A.rows());
        return false;}

    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;


    unsigned int _H_rows = _H.rows();
    for(unsigned int i = 0; i < _H_rows; ++i)
        _H(i,i) += _epsRegularisation;

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
    _A_rm = _A;
    qpOASES::returnValue val =_problem->init(_H.data(),_g.data(),
                       _A_rm.data(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(qpOASES::getSimpleStatus(val) < 0)
    {
#ifdef OPENSOT_VERBOSE
        _problem->printProperties();

        if(val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
            printConstraintsInfo();

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
    if(!(_g.size() == g.size())){
        XBot::Logger::error("g size: %i \n", g.size());
        XBot::Logger::error("should be: %i \n", _g.size());
        return false;}
    if(!(_H.cols() == H.cols())){
        XBot::Logger::error("H cols: %i \n", H.cols());
        XBot::Logger::error("should be: %i \n", _H.cols());
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
        XBot::Logger::error("A cols: %i \n", A.cols());
        XBot::Logger::error("should be: %i \n", _H.cols());
        return false;}
    if(!(lA.rows() == A.rows())){
        XBot::Logger::error("lA size: %i \n", lA.rows());
        XBot::Logger::error("A rows: %i \n", A.rows());
        return false;}
    if(!(lA.rows() == uA.rows())){
        XBot::Logger::error("lA size: %i \n", lA.rows());
        XBot::Logger::error("uA size: %i \n", uA.rows());
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

    unsigned int _H_rows = _H.rows();
    for(unsigned int i = 0; i < _H_rows; ++i)
        _H(i,i) += _epsRegularisation;

    _A_rm = _A;
    qpOASES::returnValue val =_problem->hotstart(_H.data(),_g.data(),
                       _A_rm.data(),
                        _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(val != qpOASES::SUCCESSFUL_RETURN){
#ifdef OPENSOT_VERBOSE
        XBot::Logger::warning("WARNING OPTIMIZING TASK IN HOTSTART! ERROR  %i \n", val);
        XBot::Logger::success("RETRYING INITING WITH WARMSTART \n");
#endif

        val =_problem->init(_H.data(),_g.data(),
                           _A_rm.data(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           nWSR,0,
                           _solution.data(), _dual_solution.data(),
                           _bounds.get(), _constraints.get());

        if(val != qpOASES::SUCCESSFUL_RETURN){
#ifdef OPENSOT_VERBOSE
            XBot::Logger::warning("WARNING OPTIMIZING TASK IN WARMSTART! ERROR  %i \n", val);
            XBot::Logger::success("RETRYING INITING \n");
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
        XBot::Logger::info("ERROR GETTING PRIMAL SOLUTION! ERROR %i \n", success);
#endif
        return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);
    }
    return true;
}


OpenSoT::HessianType QPOasesBackEnd::getHessianType() {return (OpenSoT::HessianType)(_problem->getHessianType());}

void QPOasesBackEnd::setHessianType(const OpenSoT::HessianType ht){_problem->setHessianType((qpOASES::HessianType)(ht));}

void QPOasesBackEnd::printConstraintsInfo()
{
    qpOASES::Constraints infeasibleConstraints;
    _problem->getConstraints(infeasibleConstraints);
    infeasibleConstraints.print();

    qpOASES::Bounds infeasibleBounds;
    _problem->getBounds(infeasibleBounds);
    infeasibleBounds.print();
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
    unsigned int constraints_size = _lA.size();
    for(unsigned int i = 0; i < constraints_size; ++i){
        if(_lA[i] < -qpOASES::INFTY)
            _lA[i] = -qpOASES::INFTY;
        if(_uA[i] > qpOASES::INFTY)
            _uA[i] = qpOASES::INFTY;
    }

    unsigned int bounds_size = _l.size();
    for(unsigned int i = 0; i < bounds_size; ++i){
        if(_l[i] < -qpOASES::INFTY)
            _l[i] = -qpOASES::INFTY;
        if(_u[i] > qpOASES::INFTY)
            _u[i] = qpOASES::INFTY;
    }
}

bool QPOasesBackEnd::setEpsRegularisation(const double eps)
{
    if(eps < 0.0)
    {
        XBot::Logger::error("Negative eps is not allowed!");
        return false;
    }

    _epsRegularisation = eps;

    _opt->epsRegularisation = _epsRegularisation;
    _problem->setOptions(*_opt);

    return true;
}
