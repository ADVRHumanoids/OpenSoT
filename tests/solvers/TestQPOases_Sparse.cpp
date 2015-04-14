#include <qpOASES/Matrices.hpp>
#include <gtest/gtest.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <yarp/math/Math.h>
#include <qpOASES.hpp>
#include <ctime>
#include <qpOASES/Utils.hpp>
#include <fstream>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/Aggregated.h>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace yarp::sig;
using namespace yarp::math;

namespace
{
        enum HessianType
        {
            HST_ZERO,                   /**< Hessian is zero matrix (i.e. LP formulation). */
            HST_IDENTITY,               /**< Hessian is identity matrix. */
            HST_POSDEF,                 /**< Hessian is (strictly) positive definite. */
            HST_POSDEF_NULLSPACE,       /**< Hessian is positive definite on null space of active bounds/constraints. */
            HST_SEMIDEF,                /**< Hessian is positive semi-definite. */
            HST_UNKNOWN                 /**< Hessian type is unknown. */
        };

             class QPOasesProblem4Test {
             public:
                 QPOasesProblem4Test(const int number_of_variables, const int number_of_constraints,
                                OpenSoT::HessianType hessian_type = OpenSoT::HST_UNKNOWN,
                                const double eps_regularisation = 2E2):
                    _problem(new qpOASES::SQProblem(number_of_variables, number_of_constraints,
                        (qpOASES::HessianType)(hessian_type))),
                    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
                    _bounds(new qpOASES::Bounds()),
                    _constraints(new qpOASES::Constraints()),
                    _nWSR(132),
                    _epsRegularisation(eps_regularisation),
                    _solution(number_of_variables), _dual_solution(number_of_variables),
                    _opt(new qpOASES::Options())
                { setDefaultOptions();}

                 ~QPOasesProblem4Test(){}

                 void setDefaultOptions()
                 {
                     qpOASES::Options opt;
                     opt.setToMPC();
                     opt.printLevel = qpOASES::PL_NONE;
                     opt.enableRegularisation = qpOASES::BT_TRUE;
                     opt.epsRegularisation *= _epsRegularisation;

                     opt.ensureConsistency();

                     _problem->setOptions(opt);

                     std::cout<<GREEN<<"Solver Default Options:"<<DEFAULT<<std::endl;
                     opt.print();

                     _opt.reset(new qpOASES::Options(opt));
                 }

                 const boost::shared_ptr<qpOASES::SQProblem>& getProblem(){return _problem;}

                 qpOASES::Options getOptions()
                 {
                     return _problem->getOptions();
                 }

                 void setOptions(const qpOASES::Options& options)
                 {
                     _opt.reset(new qpOASES::Options(options));
                     _problem->setOptions(options);
                 }

                 bool initProblem(const yarp::sig::Matrix& H, const Vector& g,
                                  const yarp::sig::Matrix& A,
                                  const Vector& lA, const Vector& uA,
                                  const Vector& l, const Vector& u)
                 {
                     _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
                     checkINFTY();


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

                 bool updateTask(const yarp::sig::Matrix& H, const Vector& g)
                 {
                     if(!(_g.size() == _H.rows())){
                         std::cout<<RED<<"g size: "<<_g.size()<<DEFAULT<<std::endl;
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
                         _H.resize(H.rows(), H.cols());
                         _H = H;
                         _g.resize(g.size());
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

                 bool updateConstraints(const yarp::sig::Matrix& A, const Vector& lA, const Vector& uA)
                 {
                     if(!(_A.cols() == A.cols())){
                         std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
                         return false;}
                     if(!(lA.size() == A.rows())){
                         std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"A rows: "<<A.rows()<<DEFAULT<<std::endl;
                         return false;}
                     if(!(lA.size() == uA.size())){
                         std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"uA size: "<<uA.size()<<DEFAULT<<std::endl;
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
                         _A.resize(A.rows(), A.cols());
                         _A = A;
                         _lA.resize(lA.size());
                         _lA = lA;
                         _uA.resize(uA.size());
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

                 bool updateBounds(const Vector& l, const Vector& u)
                 {
                     if(!(l.size() == _l.size())){
                         std::cout<<RED<<"l size: "<<l.size()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"should be: "<<_l.size()<<DEFAULT<<std::endl;
                         return false;}
                     if(!(u.size() == _u.size())){
                         std::cout<<RED<<"u size: "<<u.size()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"should be: "<<_u.size()<<DEFAULT<<std::endl;
                         return false;}
                     if(!(l.size() == u.size())){
                         std::cout<<RED<<"l size: "<<l.size()<<DEFAULT<<std::endl;
                         std::cout<<RED<<"u size: "<<u.size()<<DEFAULT<<std::endl;
                         return false;}

                     _l = l;
                     _u = u;

                     return true;
                 }

                 bool updateProblem(const yarp::sig::Matrix& H, const Vector& g,
                                    const yarp::sig::Matrix& A,
                                    const Vector& lA, const Vector& uA,
                                    const Vector& l, const Vector& u)
                 {
                     bool success = true;
                     success = success && updateBounds(l, u);
                     success = success && updateConstraints(A, lA, uA);
                     success = success && updateTask(H, g);
                     return success;
                 }

                 bool addTask(const yarp::sig::Matrix& H, const Vector& g)
                 {
                     if(H.cols() == _H.cols())
                     {
                         if(!(g.size() == H.rows())){
                             std::cout<<RED<<"g size: "<<g.size()<<DEFAULT<<std::endl;
                             std::cout<<RED<<"H rows: "<<H.rows()<<DEFAULT<<std::endl;
                             return false;}

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
                         _problem->setOptions(*_opt.get());
                         return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
                     }

                     std::cout<<RED<<"H cols: "<<H.cols()<<DEFAULT<<std::endl;
                     std::cout<<RED<<"should be: "<<_H.cols()<<DEFAULT<<std::endl;
                     return false;
                 }

                 bool addConstraints(const yarp::sig::Matrix& A, const Vector& lA, const Vector& uA)
                 {
                     if(A.cols() == _A.cols())
                     {
                         if(!(lA.size() == A.rows())){
                             std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
                             std::cout<<RED<<"A rows: "<<A.rows()<<DEFAULT<<std::endl;
                             return false;}
                         if(!(lA.size() == uA.size())){
                             std::cout<<RED<<"lA size: "<<lA.size()<<DEFAULT<<std::endl;
                             std::cout<<RED<<"uA size: "<<uA.size()<<DEFAULT<<std::endl;
                             return false;}

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
                         _problem->setOptions(*_opt.get());
                         return initProblem(_H, _g, _A, _lA, _uA, _l, _u);
                     }
                     std::cout<<RED<<"A cols: "<<A.cols()<<DEFAULT<<std::endl;
                     std::cout<<RED<<"should be: "<<_A.cols()<<DEFAULT<<std::endl;
                     return false;
                 }

                 bool solve()
                 {
                     int nWSR = _nWSR;
                     checkINFTY();

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
                         _problem->setOptions(*_opt.get());
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
                             _problem->setOptions(*_opt.get());
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

                 const Vector& getSolution(){return _solution;}

                 OpenSoT::HessianType getHessianType(){return (OpenSoT::HessianType)(_problem->getHessianType());}

                 void setHessianType(const OpenSoT::HessianType ht)
                 {_problem->setHessianType((qpOASES::HessianType)(ht));}

                 int getnWSR(){return _nWSR;}

                 void setnWSR(const int nWSR){_nWSR = nWSR;}

                 const qpOASES::Bounds& getActiveBounds(){return *_bounds;}

                 const qpOASES::Constraints& getActiveConstraints(){return *_constraints;}

                 const yarp::sig::Matrix& getH(){return _H;}

                 const yarp::sig::Vector& getg(){return _g;}

                 const yarp::sig::Matrix& getA(){return _A;}

                 const yarp::sig::Vector& getlA(){return _lA;}

                 const yarp::sig::Vector& getuA(){return _uA;}

                 const yarp::sig::Vector& getl(){return _l;}

                 const yarp::sig::Vector& getu(){return _u;}

                 void printProblemInformation(const int problem_number, const std::string problem_id)
                 {
                     std::cout<<std::endl;
                     if(problem_number == -1)
                         std::cout<<GREEN<<"PROBLEM ID: "<<DEFAULT<<problem_id<<std::endl;
                     else
                         std::cout<<GREEN<<"PROBLEM "<<problem_number<<" ID: "<<DEFAULT<<problem_id<<std::endl;
                     std::cout<<GREEN<<"eps Regularisation factor: "<<DEFAULT<<_problem->getOptions().epsRegularisation<<std::endl;
                     std::cout<<GREEN<<"# OF CONSTRAINTS: "<<DEFAULT<<_problem->getNC()<<std::endl;
                     std::cout<<GREEN<<"# OF BOUNDS: "<<DEFAULT<<_l.size()<<std::endl;
                     std::cout<<GREEN<<"# OF VARIABLES: "<<DEFAULT<<_problem->getNV()<<std::endl;
                 //    std::cout<<GREEN<<"H: "<<DEFAULT<<_H.toString()<<std::endl;
                 //    std::cout<<GREEN<<"g: "<<DEFAULT<<_g.toString()<<std::endl;
                 //    std::cout<<GREEN<<"A: "<<DEFAULT<<_A.toString()<<std::endl;
                 //    std::cout<<GREEN<<"lA: "<<DEFAULT<<_lA.toString()<<std::endl;
                 //    std::cout<<GREEN<<"uA: "<<DEFAULT<<_uA.toString()<<std::endl;
                 //    std::cout<<GREEN<<"u: "<<DEFAULT<<_u.toString()<<std::endl;
                 //    std::cout<<GREEN<<"l: "<<DEFAULT<<_l.toString()<<std::endl;
                     std::cout<<std::endl;
                 }

                 bool writeQPIntoMFile(const std::string& file_name)
                 {
                     std::ofstream file;
                     file.open(file_name);
                     if(file.is_open())
                     {
                         file<<"H = [\n"<<_H.toString()<<"\n]\n\n";
                         file<<"g = [\n"<<_g.toString()<<"\n]\n\n";
                         file<<"A = [\n"<<_A.toString()<<"\n]\n\n";
                         file<<"lA = [\n"<<_lA.toString()<<"\n]\n\n";
                         file<<"uA = [\n"<<_uA.toString()<<"\n]\n\n";
                         file<<"l = [\n"<<_l.toString()<<"\n]\n\n";
                         file<<"u = [\n"<<_u.toString()<<"\n]";

                         file.close();
                         return true;
                     }
                     return false;
                 }

             protected:
                 void checkInfeasibility()
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

                 void checkINFTY()
                 {
                     unsigned int constraints_size = _lA.size();
                     for(unsigned int i = 0; i < constraints_size; ++i){
                         if(_lA[i] < -qpOASES::INFTY)
                             _lA[i] = -qpOASES::INFTY;
                         if(_uA[i] > qpOASES::INFTY)
                             _uA[i] = qpOASES::INFTY;}

                     unsigned int bounds_size = _l.size();
                     for(unsigned int i = 0; i < bounds_size; ++i){
                         if(_l[i] < -qpOASES::INFTY)
                             _l[i] = -qpOASES::INFTY;
                         if(_u[i] > qpOASES::INFTY)
                             _u[i] = qpOASES::INFTY;}
                 }

                 boost::shared_ptr<qpOASES::SQProblem> _problem;
                 boost::shared_ptr<qpOASES::Bounds> _bounds;
                 boost::shared_ptr<qpOASES::Constraints> _constraints;
                 int _nWSR;
                 double _epsRegularisation;
                 yarp::sig::Matrix _H;
                 Vector _g;
                 yarp::sig::Matrix _A;
                 Vector _lA;
                 Vector _uA;
                 Vector _l;
                 Vector _u;
                 Vector _solution;
                 Vector _dual_solution;
                 boost::shared_ptr<qpOASES::Options> _opt;
             };

             class QPOases_sot4Test: public OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>
             {
             public:
                 typedef boost::shared_ptr<QPOases_sot4Test> Ptr;

                 QPOases_sot4Test(Stack& stack_of_tasks, const double eps_regularisation = 2E2):
                     Solver(stack_of_tasks),
                     _epsRegularisation(eps_regularisation)
                 {
                     if(!prepareSoT())
                         throw "Can Not initizalize SoT!";
                 }

                 QPOases_sot4Test(Stack& stack_of_tasks,
                                 ConstraintPtr bounds,
                                 const double eps_regularisation = 2E2):
                     Solver(stack_of_tasks, bounds),
                     _epsRegularisation(eps_regularisation)
                 {
                     if(!prepareSoT())
                         throw "Can Not initizalize SoT with bounds!";
                 }

                 ~QPOases_sot4Test(){}

                 bool solve(Vector& solution)
                 {
                     for(unsigned int i = 0; i < _tasks.size(); ++i)
                     {
                         yarp::sig::Matrix H;
                         yarp::sig::Vector g;
                         computeVelCtrlCostFunction(_tasks[i], H, g);
                         if(!_qp_stack_of_tasks[i].updateTask(H, g))
                             return false;

                         OpenSoT::constraints::Aggregated::Ptr constraints_task_i(new OpenSoT::constraints::Aggregated(_tasks[i]->getConstraints(), _tasks[i]->getXSize()));
                         yarp::sig::Matrix A = constraints_task_i->getAineq();
                         yarp::sig::Vector lA = constraints_task_i->getbLowerBound();
                         yarp::sig::Vector uA = constraints_task_i->getbUpperBound();
                         if(i > 0)
                         {
                             for(unsigned int j = 0; j < i; ++j)
                             {
                                 yarp::sig::Matrix tmp_A;
                                 yarp::sig::Vector tmp_lA, tmp_uA;
                                 computeVelCtrlOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A, tmp_lA, tmp_uA);
                                 A = yarp::math::pile(A, tmp_A);
                                 lA = yarp::math::cat(lA, tmp_lA);
                                 uA = yarp::math::cat(uA, tmp_uA);
                             }
                         }
                         if(!_qp_stack_of_tasks[i].updateConstraints(A, lA, uA))
                             return false;

                         if(_bounds){
                             constraints_task_i = OpenSoT::constraints::Aggregated::Ptr(new OpenSoT::constraints::Aggregated(constraints_task_i, _bounds, _tasks[i]->getXSize()));
                             if(!_qp_stack_of_tasks[i].updateBounds(constraints_task_i->getLowerBound(), constraints_task_i->getUpperBound()))
                                 return false;}

                         if(!_qp_stack_of_tasks[i].solve())
                             return false;

                         solution = _qp_stack_of_tasks[i].getSolution();
                     }
                     return true;
                 }

                 unsigned int getNumberOfTasks(){return _qp_stack_of_tasks.size();}

                 bool setOptions(const unsigned int i, const qpOASES::Options &opt)
                 {
                     if(i > _qp_stack_of_tasks.size()){
                         std::cout<<RED<<"ERROR Index out of range!"<<DEFAULT<<std::endl;
                         return false;}

                     _qp_stack_of_tasks[i].setOptions(opt);
                     return true;
                 }

                 bool getOptions(const unsigned int i, qpOASES::Options& opt)
                 {

                     if(i > _qp_stack_of_tasks.size()){
                         std::cout<<RED<<"Index out of range!"<<DEFAULT<<std::endl;
                         return false;}

                     opt = _qp_stack_of_tasks[i].getOptions();
                     return true;
                 }

             protected:
                 vector <QPOasesProblem4Test> _qp_stack_of_tasks;

                 double _epsRegularisation;

                 bool prepareSoT()
                 {
                     for(unsigned int i = 0; i < _tasks.size(); ++i)
                     {
                         yarp::sig::Matrix H;
                         yarp::sig::Vector g;
                         computeVelCtrlCostFunction(_tasks[i], H, g);

                         OpenSoT::constraints::Aggregated::Ptr constraints_task_i(new OpenSoT::constraints::Aggregated(_tasks[i]->getConstraints(), _tasks[i]->getXSize()));
                         yarp::sig::Matrix A = constraints_task_i->getAineq();
                         yarp::sig::Vector lA = constraints_task_i->getbLowerBound();
                         yarp::sig::Vector uA = constraints_task_i->getbUpperBound();
                         if(i > 0)
                         {
                             for(unsigned int j = 0; j < i; ++j)
                             {
                                 yarp::sig::Matrix tmp_A;
                                 yarp::sig::Vector tmp_lA, tmp_uA;
                                 computeVelCtrlOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A, tmp_lA, tmp_uA);
                                 A = pile(A, tmp_A);
                                 lA = cat(lA, tmp_lA);
                                 uA = cat(uA, tmp_uA);
                             }
                         }

                         if(_bounds)
                             constraints_task_i = OpenSoT::constraints::Aggregated::Ptr(new OpenSoT::constraints::Aggregated(constraints_task_i, _bounds, _tasks[i]->getXSize()));
                         yarp::sig::Vector l = constraints_task_i->getLowerBound();
                         yarp::sig::Vector u = constraints_task_i->getUpperBound();

                         QPOasesProblem4Test problem_i(_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
                                                  _epsRegularisation);

                         if(problem_i.initProblem(H, g, A, lA, uA, l, u)){
                             _qp_stack_of_tasks.push_back(problem_i);
                             _qp_stack_of_tasks[i].printProblemInformation(i, _tasks[i]->getTaskID());}
                         else{
                             std::cout<<RED<<"ERROR: INITIALIZING STACK "<<i<<DEFAULT<<std::endl;
                             return false;}
                     }
                     return true;
                 }

                 void computeVelCtrlCostFunction(const TaskPtr& task, yarp::sig::Matrix& H, yarp::sig::Vector& g)
                 {
                     H = task->getA().transposed() * task->getWeight() * task->getA();
                     g = -1.0 * task->getLambda() * task->getA().transposed() * task->getWeight() * task->getb();
                 }

                 void computeVelCtrlOptimalityConstraint(const TaskPtr& task, QPOasesProblem4Test& problem,
                                        yarp::sig::Matrix& A, yarp::sig::Vector& lA, yarp::sig::Vector& uA)
                 {
                     OpenSoT::constraints::BilateralConstraint::Ptr optimality_bilateral_constraint(
                         new OpenSoT::constraints::BilateralConstraint(
                                 task->getA(),
                                 task->getA()*problem.getSolution(),
                                 task->getA()*problem.getSolution()));
                     A = optimality_bilateral_constraint->getAineq();
                     lA = optimality_bilateral_constraint->getbLowerBound();
                     uA = optimality_bilateral_constraint->getbUpperBound();
                 }
             };


    class testqpOASESSparseMatrices: public ::testing::Test
    {
    protected:

        testqpOASESSparseMatrices()
        {

        }

        virtual ~testqpOASESSparseMatrices() {

        }

        virtual void SetUp() {

        }

        virtual void TearDown() {

        }

    };

    TEST_F(testqpOASESSparseMatrices, testSparseMatrices)
    {
        yarp::sig::Matrix sym_sparse_mat_yarp(4,4);
        sym_sparse_mat_yarp.eye();
        for(unsigned int i = 0; i < sym_sparse_mat_yarp.rows(); ++i)
            sym_sparse_mat_yarp(i,i) = double(i);
        sym_sparse_mat_yarp(0,3) = 5.0;
        sym_sparse_mat_yarp(3,0) = 5.0;

        qpOASES::SymSparseMat sym_sparse_mat_qpoases(sym_sparse_mat_yarp.rows(), sym_sparse_mat_yarp.cols(),
                                                     sym_sparse_mat_yarp.rows(), sym_sparse_mat_yarp.data());

        std::cout<<"SymSparseMat1:"<<std::endl;
        sym_sparse_mat_qpoases.print();

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::sparse_int_t H_jc[] = { 0,  4,  8, 12, 16, 20, 20, 20, 20, 20, 20,
                               24, 28, 32, 36, 40, 40, 40, 40, 40, 40,
                               44, 48, 52, 56, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
                               64, 68, 72, 76, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80 };

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::real_t H_val[] = {10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1,
            1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 1,
            10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 1, 10, 1,
            1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10};

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::sparse_int_t H_ir[] = {
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38};

        std::cout<<std::endl;
        qpOASES::SymSparseMat sym_sparse_mat_qpoases2(180, 180, H_ir, H_jc, H_val);
        std::cout<<"SymSparseMat2:"<<std::endl;
        sym_sparse_mat_qpoases2.print();


        std::cout<<std::endl;
        yarp::sig::Matrix sym_sparse_mat_yarp2(3,3);
        sym_sparse_mat_yarp2.eye();
        for(unsigned int i = 0; i < sym_sparse_mat_yarp2.rows(); ++i)
            sym_sparse_mat_yarp2(i,i) = double(i);
        sym_sparse_mat_yarp2(0,2) = 5.0;
        sym_sparse_mat_yarp2(2,0) = 5.0;

        qpOASES::SymSparseMat sym_sparse_mat_qpoases3(sym_sparse_mat_yarp2.rows(), sym_sparse_mat_yarp2.cols(),
                                                     3, sym_sparse_mat_yarp2.data());
        std::cout<<"SymSparseMat3:"<<std::endl;
        sym_sparse_mat_qpoases3.print();


        yarp::sig::Matrix sparse_mat_yarp(2,4);
        sparse_mat_yarp.zero();
        sparse_mat_yarp(0,0) = 5.0;
        sparse_mat_yarp(0,2) = 3.0;
        sparse_mat_yarp(1,0) = 6.0;
        sparse_mat_yarp(1,2) = 7.0;

        qpOASES::SparseMatrix sparse_mat_qpoases(sparse_mat_yarp.rows(), sparse_mat_yarp.cols(),
                                                 sparse_mat_yarp.cols(), sparse_mat_yarp.data());

        std::cout<<"SparseMat1:"<<std::endl;
        sparse_mat_qpoases.print();

        yarp::sig::Matrix sparse_mat_yarp2(5,3);
        sparse_mat_yarp2.zero();
        sparse_mat_yarp2(0,0) = 5.0;
        sparse_mat_yarp2(1,0) = 3.0;
        sparse_mat_yarp2(2,0) = 7.0;
        sparse_mat_yarp2(3,0) = 8.0;

        qpOASES::SparseMatrix sparse_mat_qpoases2(sparse_mat_yarp2.rows(), sparse_mat_yarp2.cols(),
                                                 sparse_mat_yarp2.cols(), sparse_mat_yarp2.data());

        std::cout<<"SparseMat2:"<<std::endl;
        sparse_mat_qpoases2.print();
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
