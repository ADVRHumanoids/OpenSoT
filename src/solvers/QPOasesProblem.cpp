#include <OpenSoT/solvers/QPOasesProblem.h>
#include <qpOASES.hpp>
#include <ctime>
#include <qpOASES/Utils.hpp>
#include <fstream>
#include <boost/make_shared.hpp>
#include <qpOASES/Matrices.hpp>
#include <OpenSoT/utils/math/Math.h>


#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;
using namespace OpenSoT::utils::math;

QPOasesProblem::QPOasesProblem(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation):
    _problem(new qpOASES::SQProblem(number_of_variables,
                                    number_of_constraints,
                                    (qpOASES::HessianType)(hessian_type))),
    _bounds(new qpOASES::Bounds()),
    _constraints(new qpOASES::Constraints()),
    _nWSR(132),
    _epsRegularisation(eps_regularisation),
    _solution(number_of_variables), _dual_solution(number_of_variables),
    _opt(new qpOASES::Options())
{
    _H.setZero(0,0);
    _g.setZero(0);
    _A.setZero(0,0);
    _lA.setZero(0);
    _uA.setZero(0);
    _l.setZero(0);
    _u.setZero(0);
    setDefaultOptions();}

QPOasesProblem::~QPOasesProblem()
{}

void QPOasesProblem::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToMPC();
    opt.printLevel = qpOASES::PL_NONE;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= _epsRegularisation;
    opt.numRegularisationSteps = 2;
    opt.numRefinementSteps = 1;
    opt.enableFlippingBounds = qpOASES::BT_TRUE;

    opt.ensureConsistency();

    _problem->setOptions(opt);

    std::cout<<GREEN<<"Solver Default Options:"<<DEFAULT<<std::endl;
    opt.print();

    _opt.reset(new qpOASES::Options(opt));
}

void QPOasesProblem::setOptions(const qpOASES::Options &options){
    _opt.reset(new qpOASES::Options(options));
    _problem->setOptions(options);}

qpOASES::Options QPOasesProblem::getOptions(){
    return _problem->getOptions();}

bool QPOasesProblem::initProblem(const MatrixXd &H, const Eigen::VectorXd &g,
                                 const MatrixXd &A,
                                 const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                 const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
    checkINFTY();


    if(!(_l.rows() == _u.rows())){
        std::cout<<RED<<"l size: "<<_l.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"u size: "<<_u.rows()<<DEFAULT<<std::endl;
        assert(_l.rows() == _u.rows());}
    if(!(_lA.rows() == _A.rows())){
        std::cout<<RED<<"lA size: "<<_lA.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"A rows: "<<_A.rows()<<DEFAULT<<std::endl;
        assert(_lA.rows() == _A.rows());}
    if(!(_lA.rows() == _uA.rows())){
        std::cout<<RED<<"lA size: "<<_lA.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"uA size: "<<_uA.rows()<<DEFAULT<<std::endl;
        assert(_lA.rows() == _uA.rows());}

    int nWSR = _nWSR;
    H_sparse.reset(new qpOASES::SymSparseMat(_H.rows(), _H.cols(), _H.rows(), _H.data()));
    H_sparse->createDiagInfo();
    A_dense.reset();
    if(!(_A.data() == NULL))
        A_dense = boost::make_shared<qpOASES::DenseMatrix>(
                qpOASES::DenseMatrix(_A.rows(), _A.cols(), _A.cols(), _A.data()));

        qpOASES::returnValue val =_problem->init(H_sparse.get(),_g.data(),
                       A_dense.get(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(val != qpOASES::SUCCESSFUL_RETURN)
    {
        _problem->printProperties();

        if(val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
            checkInfeasibility();

        std::cout<<RED<<"ERROR INITIALIZING QP PROBLEM "<<DEFAULT<<std::endl;
        std::cout<<RED<<"CODE ERROR: "<<val<<DEFAULT<<std::endl;


#ifndef NDEBUG //Log is generated only in DEBUG Mode!
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [80];
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (buffer,80,"%Y%m%d_%H%M%S",timeinfo);
        std::string file_name = "qp_problem_log_";
        file_name.append(buffer);
        file_name = file_name + ".m";
        if(writeQPIntoMFile(file_name.c_str()))
            std::cout<<"Wrote QP problem into mat file "<<file_name<<std::endl;
        else
            std::cout<<RED<<"ERROR while writing QP problem into mat file!"<<DEFAULT<<std::endl;
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
        std::cout<<RED<<"ERROR GETTING PRIMAL SOLUTION IN INITIALIZATION! ERROR "<<success<<DEFAULT<<std::endl;
        return false;}
    return true;
}

bool QPOasesProblem::updateTask(const MatrixXd &H, const Eigen::VectorXd &g)
{
    if(!(_g.rows() == _H.rows())){
        std::cout<<RED<<"g size: "<<_g.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"H rows: "<<_H.rows()<<DEFAULT<<std::endl;
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

bool QPOasesProblem::updateConstraints(const MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA)
{
    if(!(_A.cols() == A.cols())){
        std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
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

bool QPOasesProblem::updateBounds(const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    if(!(l.rows() == _l.rows())){
        std::cout<<RED<<"l size: "<<l.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_l.rows()<<DEFAULT<<std::endl;
        return false;}
    if(!(u.rows() == _u.rows())){
        std::cout<<RED<<"u size: "<<u.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_u.rows()<<DEFAULT<<std::endl;
        return false;}
    if(!(l.rows() == u.rows())){
        std::cout<<RED<<"l size: "<<l.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"u size: "<<u.rows()<<DEFAULT<<std::endl;
        return false;}

    _l = l;
    _u = u;

    return true;
}

bool QPOasesProblem::updateProblem(const MatrixXd &H, const Eigen::VectorXd &g,
                                   const MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                   const Eigen::VectorXd &l, const Eigen::VectorXd &u)
{
    bool success = true;
    success = success && updateBounds(l, u);
    success = success && updateConstraints(A, lA, uA);
    success = success && updateTask(H, g);
    return success;
}

bool QPOasesProblem::addTask(const MatrixXd &H, const Eigen::VectorXd &g)
{
    if(H.cols() == _H.cols())
    {
        if(!(g.rows() == H.rows())){
            std::cout<<RED<<"g size: "<<g.rows()<<DEFAULT<<std::endl;
            std::cout<<RED<<"H rows: "<<H.rows()<<DEFAULT<<std::endl;
            return false;}

        pile(_H,H);
        pile(_g,g);

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

    std::cout<<RED<<"H cols: "<<H.cols()<<DEFAULT<<std::endl;
    std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
    return false;
}

bool QPOasesProblem::addConstraints(const MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA)
{
    if(A.cols() == _A.cols())
    {
        if(!(lA.rows() == A.rows())){
            std::cout<<RED<<"lA size: "<<lA.rows()<<DEFAULT<<std::endl;
            std::cout<<RED<<"A rows: "<<A.rows()<<DEFAULT<<std::endl;
            return false;}
        if(!(lA.rows() == uA.rows())){
            std::cout<<RED<<"lA size: "<<lA.rows()<<DEFAULT<<std::endl;
            std::cout<<RED<<"uA size: "<<uA.rows()<<DEFAULT<<std::endl;
            return false;}

        pile(_A,A);
        pile(_lA,lA);
        pile(_uA,uA);

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
    std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
    std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
    return false;
}

bool QPOasesProblem::solve()
{
    int nWSR = _nWSR;
    checkINFTY();

    H_sparse.reset(new qpOASES::SymSparseMat(_H.rows(), _H.cols(), _H.rows(), _H.data()));
    H_sparse->createDiagInfo();
    A_dense.reset();
    if(!(_A.data() == NULL))
        A_dense = boost::make_shared<qpOASES::DenseMatrix>(
                    qpOASES::DenseMatrix(_A.rows(), _A.cols(), _A.cols(), _A.data()));


    qpOASES::returnValue val =_problem->hotstart(H_sparse.get(),_g.data(),
                       A_dense.get(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(val != qpOASES::SUCCESSFUL_RETURN){
        std::cout<<YELLOW<<"WARNING OPTIMIZING TASK IN HOTSTART! ERROR "<<val<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"RETRYING INITING WITH WARMSTART"<<DEFAULT<<std::endl;

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _problem->getNV();
        int number_of_constraints = _problem->getNC();

        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        _problem->setOptions(*_opt.get());
        val =_problem->init(H_sparse.get(),_g.data(),
                           A_dense.get(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           nWSR,0,
                           _solution.data(), _dual_solution.data(),
                           _bounds.get(), _constraints.get());

        if(val != qpOASES::SUCCESSFUL_RETURN){
            std::cout<<YELLOW<<"WARNING OPTIMIZING TASK IN WARMSTART! ERROR "<<val<<DEFAULT<<std::endl;
            std::cout<<GREEN<<"RETRYING INITING"<<DEFAULT<<std::endl;

            _problem.reset();
            _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                                  number_of_variables,
                                                                  number_of_constraints,
                                                                  hessian_type));
            _problem->setOptions(*_opt.get());
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

    if(success != qpOASES::SUCCESSFUL_RETURN){
        std::cout<<"ERROR GETTING PRIMAL SOLUTION! ERROR "<<success<<std::endl;
        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _problem->getNV();
        int number_of_constraints = _problem->getNC();
        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);
    }
    return true;
}


OpenSoT::HessianType QPOasesProblem::getHessianType() {return (OpenSoT::HessianType)(_problem->getHessianType());}

void QPOasesProblem::setHessianType(const OpenSoT::HessianType ht){_problem->setHessianType((qpOASES::HessianType)(ht));}

void QPOasesProblem::checkInfeasibility()
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

void QPOasesProblem::printProblemInformation(const int problem_number, const std::string& problem_id,
                                             const std::string& constraints_id, const std::string& bounds_id)
{
    std::cout<<std::endl;
    if(problem_number == -1)
        std::cout<<GREEN<<"PROBLEM ID: "<<DEFAULT<<problem_id<<std::endl;
    else
        std::cout<<GREEN<<"PROBLEM "<<problem_number<<" ID: "<<DEFAULT<<problem_id<<std::endl;
    std::cout<<GREEN<<"eps Regularisation factor: "<<DEFAULT<<_problem->getOptions().epsRegularisation<<std::endl;
    std::cout<<GREEN<<"CONSTRAINTS ID: "<<DEFAULT<<constraints_id<<std::endl;
    std::cout<<GREEN<<"     # OF CONSTRAINTS: "<<DEFAULT<<_problem->getNC()<<std::endl;
    std::cout<<GREEN<<"BOUNDS ID: "<<DEFAULT<<bounds_id<<std::endl;
    std::cout<<GREEN<<"     # OF BOUNDS: "<<DEFAULT<<_l.rows()<<std::endl;
    std::cout<<GREEN<<"# OF VARIABLES: "<<DEFAULT<<_problem->getNV()<<std::endl;
//    std::cout<<GREEN<<"H: "<<DEFAULT<<_H<<std::endl;
//    std::cout<<GREEN<<"g: "<<DEFAULT<<_g<<std::endl;
//    std::cout<<GREEN<<"A: "<<DEFAULT<<_A<<std::endl;
//    std::cout<<GREEN<<"lA: "<<DEFAULT<<_lA<<std::endl;
//    std::cout<<GREEN<<"uA: "<<DEFAULT<<_uA<<std::endl;
//    std::cout<<GREEN<<"u: "<<DEFAULT<<_u<<std::endl;
//    std::cout<<GREEN<<"l: "<<DEFAULT<<_l<<std::endl;
    std::cout<<std::endl;
}

bool QPOasesProblem::writeQPIntoMFile(const std::string& file_name)
{
    std::ofstream file(file_name.c_str());
    if(file.is_open())
    {
        file<<"H = [\n"<<_H<<"\n]\n\n";
        file<<"g = [\n"<<_g<<"\n]\n\n";
        file<<"A = [\n"<<_A<<"\n]\n\n";
        file<<"lA = [\n"<<_lA<<"\n]\n\n";
        file<<"uA = [\n"<<_uA<<"\n]\n\n";
        file<<"l = [\n"<<_l<<"\n]\n\n";
        file<<"u = [\n"<<_u<<"\n]";

        file.close();
        return true;
    }
    return false;
}

void QPOasesProblem::checkINFTY()
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
