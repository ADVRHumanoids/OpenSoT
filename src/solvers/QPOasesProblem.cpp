#include <OpenSoT/solvers/QPOasesProblem.h>
#include <yarp/math/Math.h>
#include <qpOASES.hpp>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;
using namespace yarp::math;

QPOasesProblem::QPOasesProblem(const int number_of_variables,
                               const int number_of_constraints,
                               OpenSoT::HessianType hessian_type, const double eps_regularisation):
    _problem(new qpOASES::SQProblem(number_of_variables,
                                    number_of_constraints,
                                    (qpOASES::HessianType)(hessian_type))),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(new qpOASES::Bounds()),
    _constraints(new qpOASES::Constraints()),
    _nWSR(132),
    _epsRegularisation(eps_regularisation),
    _solution(number_of_variables), _dual_solution(number_of_variables)
{ setDefaultOptions();}

QPOasesProblem::~QPOasesProblem()
{}

void QPOasesProblem::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToMPC();
    opt.printLevel = qpOASES::PL_LOW;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= _epsRegularisation;

    opt.ensureConsistency();

    _problem->setOptions(opt);

    std::cout<<GREEN<<"Solver Default Options:"<<DEFAULT<<std::endl;
    opt.print();
}

void QPOasesProblem::setOptions(const qpOASES::Options &options){
    _problem->setOptions(options);}

qpOASES::Options QPOasesProblem::getOptions(){
    return _problem->getOptions();}

bool QPOasesProblem::initProblem(const Matrix &H, const Vector &g,
                                 const Matrix &A,
                                 const Vector &lA, const Vector &uA,
                                 const Vector &l, const Vector &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;


    if(!(_l.size() == _u.size())){
        std::cout<<RED<<"l size: "<<_l.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"u size: "<<_u.size()<<DEFAULT<<std::endl;
        assert(_l.size() == _u.size());}
    if(!(_lA.size() == _A.rows())){
        std::cout<<RED<<"lA size: "<<_lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"A rows: "<<_A.rows()<<DEFAULT<<std::endl;
        assert(_lA.size() == _A.rows());}
    if(!(_lA.size() == _uA.size())){
        std::cout<<RED<<"lA size: "<<_lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"uA size: "<<_uA.size()<<DEFAULT<<std::endl;
        assert(_lA.size() == _uA.size());}

    int nWSR = _nWSR;
        qpOASES::returnValue val =_problem->init( _H.data(),_g.data(),
                       _A.data(),
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
        return false;
    }

    if(_solution.size() != _problem->getNV())
        _solution.resize(_problem->getNV());

    if(_dual_solution.size() != _problem->getNV() + _problem->getNC())
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

void QPOasesProblem::updateTask(const Matrix &H, const Vector &g)
{
    //For now we do not allow change in cost function size!
    ///TODO: allow at least change in rows both for H and g!
    if(!(_H.rows() == H.rows())){
        std::cout<<RED<<"H rows: "<<H.rows()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_H.rows()<<DEFAULT<<std::endl;
        assert(_H.rows() == H.rows());}
    if(!(_H.cols() == H.cols())){
        std::cout<<RED<<"H cols: "<<H.cols()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
        assert(_H.cols() == H.cols());}
    if(!(_g.size() == g.size())){
        std::cout<<RED<<"g size: "<<g.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_g.size()<<DEFAULT<<std::endl;
        assert(_g.size() == g.size());}
    if(!(_g.size() == _H.rows())){
        std::cout<<RED<<"g size: "<<_g.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"H rows: "<<_H.rows()<<DEFAULT<<std::endl;
        assert(_g.size() == _H.rows());}

    _H = H;
    _g = g;
}

void QPOasesProblem::updateConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if(!(_A.cols() == _A.cols())){
        std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
        assert(_A.cols() == _A.cols());}
    if(!(lA.size() == A.rows())){
        std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"A rows: "<<A.rows()<<DEFAULT<<std::endl;
        assert(lA.size() == A.rows());}
    if(!(lA.size() == uA.size())){
        std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"uA size: "<<uA.size()<<DEFAULT<<std::endl;
        assert(lA.size() == uA.size());}

    if(_A.rows() != A.rows())
        _A.resize(A.rows(), A.cols());
    if(_lA.size() != lA.size())
        _lA.resize(lA.size());
    if(_uA.size() != uA.size())
        _uA.resize(uA.size());

    _A = A;
    _lA = lA;
    _uA = uA;
}

void QPOasesProblem::updateBounds(const Vector &l, const Vector &u)
{
    if(!(l.size() == _l.size())){
        std::cout<<RED<<"l size: "<<l.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_l.size()<<DEFAULT<<std::endl;
        assert(l.size() == _l.size());}
    if(!(u.size() == _u.size())){
        std::cout<<RED<<"u size: "<<u.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"should be: "<<_u.size()<<DEFAULT<<std::endl;
        assert(u.size() == _u.size());}
    if(!(l.size() == u.size())){
        std::cout<<RED<<"l size: "<<l.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"u size: "<<u.size()<<DEFAULT<<std::endl;
        assert(l.size() == u.size());}

    _l = l;
    _u = u;
}

void QPOasesProblem::updateProblem(const Matrix &H, const Vector &g,
                                   const Matrix &A, const Vector &lA, const Vector &uA,
                                   const Vector &l, const Vector &u)
{
    updateBounds(l, u);
    updateConstraints(A, lA, uA);
    updateTask(H, g);
}

bool QPOasesProblem::addTask(const Matrix &H, const Vector &g)
{
    if(H.cols() == _H.cols())
    {
        ///TODO check rows of H and g
        _H = pile(_H, H);
        _g = cat(_g, g);

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _H.cols();
        int number_of_constraints = _A.rows();
        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        setDefaultOptions();
        return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
    }

    std::cout<<RED<<"H cols: "<<H.cols()<<DEFAULT<<std::endl;
    std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
    return false;
}

bool QPOasesProblem::addConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if(A.cols() == _A.cols())
    {
        ///TODO check rows of A, lA and uA
        _A = pile(_A, A);
        _lA = cat(_lA, lA);
        _uA = cat(_uA, uA);

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _H.cols();
        int number_of_constraints = _A.rows();
        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        setDefaultOptions();
        return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
    }
    std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
    std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
    return false;
}

bool QPOasesProblem::addBounds(const Vector &l, const Vector &u)
{
    ///TODO check size of l and u
    _l = cat(_l, l);
    _u = cat(_u, u);
    qpOASES::HessianType hessian_type = _problem->getHessianType();
    int number_of_variables = _H.cols();
    int number_of_constraints = _A.rows();
    _problem.reset();
    _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                          number_of_variables,
                                                          number_of_constraints,
                                                          hessian_type));
    setDefaultOptions();
    return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
}

bool QPOasesProblem::addProblem(const Matrix &H, const Vector &g,
                                const Matrix &A, const Vector &lA,
                                const Vector &uA, const Vector &l,
                                const Vector &u)
{
    return addTask(H, g) && addConstraints(A, lA, uA) && addBounds(l, u);
}

bool QPOasesProblem::solve()
{
    if(!(_lA.size() == _uA.size())){
        std::cout<<RED<<"_lA size: "<<_lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"_uA size: "<<_uA.size()<<DEFAULT<<std::endl;
        assert(_lA.size() == _uA.size());}
    if(!(_lA.size() == _A.rows())){
        std::cout<<RED<<"_lA size: "<<_lA.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"_A rows: "<<_A.rows()<<DEFAULT<<std::endl;
        assert(_lA.size() == _A.rows());}
    if(!(_l.size() == _u.size())){
        std::cout<<RED<<"_l size: "<<_l.size()<<DEFAULT<<std::endl;
        std::cout<<RED<<"_u size: "<<_u.size()<<DEFAULT<<std::endl;
        assert(_l.size() == _u.size());}

    int nWSR = _nWSR;

    qpOASES::returnValue val =_problem->hotstart(_H.data(),_g.data(),
                       _A.data(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    if(val != qpOASES::SUCCESSFUL_RETURN){
        std::cout<<YELLOW<<"WARNING OPTIMIZING TASK IN HOTSTART! ERROR "<<val<<DEFAULT<<std::endl;
        std::cout<<YELLOW<<"RETRYING INITING WITH INITIAL GUESS"<<DEFAULT<<std::endl;

        qpOASES::HessianType hessian_type = _problem->getHessianType();
        int number_of_variables = _problem->getNV();
        int number_of_constraints = _problem->getNC();

        _problem.reset();
        _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                              number_of_variables,
                                                              number_of_constraints,
                                                              hessian_type));
        val =_problem->init(_H.data(),_g.data(),
                           _A.data(),
                           _l.data(), _u.data(),
                           _lA.data(),_uA.data(),
                           nWSR,0,
                           _solution.data(), _dual_solution.data(),
                           _bounds.get(), _constraints.get());

        if(val != qpOASES::SUCCESSFUL_RETURN){
            std::cout<<RED<<"ERROR OPTIMIZING TASK WITH INITIAL GUESS! ERROR "<<val<<DEFAULT<<std::endl;
            std::cout<<RED<<"RETRYING INITING"<<DEFAULT<<std::endl;

            _problem.reset();
            _problem = boost::shared_ptr<qpOASES::SQProblem> (new qpOASES::SQProblem(
                                                                  number_of_variables,
                                                                  number_of_constraints,
                                                                  hessian_type));
            return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);}
    }

    // If solution has changed of size we update the size
    if(_solution.size() != _problem->getNV())
        _solution.resize(_problem->getNV());

    if(_dual_solution.size() != _problem->getNV() + _problem->getNC())
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

void OpenSoT::solvers::QPOasesProblem::checkInfeasibility()
{
    qpOASES::Constraints infeasibleConstraints;
    _problem->getConstraints(infeasibleConstraints);
    yarp::sig::Vector indices_infeasible_lower;
    yarp::sig::Vector indices_infeasible_upper;
    for(unsigned int i = 0; i < infeasibleConstraints.getNC(); ++i){
        if(infeasibleConstraints.getStatus(i) == qpOASES::ST_INFEASIBLE_LOWER)
            indices_infeasible_lower.push_back(i);
        if(infeasibleConstraints.getStatus(i) == qpOASES::ST_INFEASIBLE_UPPER)
            indices_infeasible_upper.push_back(i);
    }
    std::cout<<RED<<"Indices of LOWER INFEASIBLE CONSTRAINTS"<<DEFAULT<<std::endl;
    std::cout<<indices_infeasible_lower.toString()<<std::endl<<std::endl;
    std::cout<<RED<<"Indices of UPPER INFEASIBLE CONSTRAINTS"<<DEFAULT<<std::endl;
    std::cout<<indices_infeasible_upper.toString()<<std::endl<<std::endl;
    std::cout<<RED<<"Constraints:"<<DEFAULT<<std::endl;
    infeasibleConstraints.print();

    std::cout<<"--------------------------------------------"<<std::endl;
    for(unsigned int i = 0; i < _lA.size(); ++i)
        std::cout<<i<<": "<<_lA[i]<<" <= "<<_A.getRow(i).toString()<<" <= "<<_uA[i]<<std::endl;
}
