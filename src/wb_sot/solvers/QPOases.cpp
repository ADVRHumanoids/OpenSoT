#include <wb_sot/solvers/QPOases.h>
#include <yarp/math/Math.h>
#include <wb_sot/bounds/BilateralConstraint.h>


using namespace yarp::math;

using namespace wb_sot::solvers;

/// QPOasesProblem ///

QPOasesProblem::QPOasesProblem():
    _problem(),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(),
    _constraints(),
    _nWSR(132),
    _solution(0), _dual_solution(0),
    _is_initialized(false)
{

}

QPOasesProblem::QPOasesProblem(const int number_of_variables,
                               const int number_of_constraints,
                               qpOASES::HessianType hessian_type):
    _problem(number_of_variables, number_of_constraints, hessian_type),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(),
    _constraints(),
    _nWSR(132),
    _solution(number_of_variables), _dual_solution(number_of_variables),
    _is_initialized(false)
{
    qpOASES::Options opt;
    opt.printLevel = qpOASES::PL_HIGH;
    opt.setToReliable();
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= 2E2;
    _problem.setOptions(opt);
}

void QPOasesProblem::setProblem(const qpOASES::SQProblem &problem)
{
    _problem = problem;
}

void QPOasesProblem::setOptions(const qpOASES::Options &options)
{
    _problem.setOptions(options);
}

void QPOasesProblem::hack()
{
    if(_l.size() > 0)
        _l_ptr = _l.data();
    else
        _l_ptr = NULL;
    if(_u.size() > 0)
        _u_ptr = _u.data();
    else
        _u_ptr = NULL;
    if(_lA.size() > 0)
        _lA_ptr = _lA.data();
    else
        _lA_ptr = NULL;
    if(_uA.size() > 0)
        _uA_ptr = _uA.data();
    else
        _uA_ptr = NULL;
    if(_A.rows() > 0)
        _A_ptr = _A.data();
    else
        _A_ptr = NULL;
}

bool QPOasesProblem::initProblem(const Matrix &H, const Vector &g,
                                 const Matrix &A,
                                 const Vector &lA, const Vector &uA,
                                 const Vector &l, const Vector &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

    hack(); //<- PAY ATTENTION!

    int nWSR = _nWSR;
        qpOASES::returnValue val =_problem.init( _H.data(),_g.data(),
                       _A_ptr,
                       _l_ptr, _u_ptr,
                       _lA_ptr,_uA_ptr,
                       nWSR,0);

    _is_initialized = true;

    if(!(val == qpOASES::SUCCESSFUL_RETURN))
    {
        _is_initialized = false;
        std::cout<<"ERROR INITIALIZING QP PROBLEM "<<std::endl;
        return _is_initialized;
    }

    if(_solution.size() != _problem.getNV())
        _solution.resize(_problem.getNV());

    if(_dual_solution.size() != _problem.getNV() + _problem.getNC())
        _dual_solution.resize(_problem.getNV() + _problem.getNC());  

    //We get the solution
    int success = _problem.getPrimalSolution(_solution.data());
    _problem.getDualSolution(_dual_solution.data());
    _problem.getBounds(_bounds);
    _problem.getConstraints(_constraints);

    if(success == qpOASES::RET_QP_NOT_SOLVED ||
      (success != qpOASES::RET_QP_SOLVED &&
       success != qpOASES::SUCCESSFUL_RETURN))
    {
        std::cout<<"ERROR OPTIMIZING TASK! ERROR "<<success<<std::endl;
        _is_initialized = false;
    }
    return _is_initialized;
}

bool QPOasesProblem::updateTask(const Matrix &H, const Vector &g)
{
    if( (H.rows() == _H.rows() && H.cols() == _H.cols()) &&
         g.size() == _g.size() && _is_initialized)
    {
        _H = H;
        _g = g;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if( (A.rows() == _A.rows() && A.cols() == _A.cols()) &&
         lA.size() == _lA.size() &&
         uA.size() == _uA.size() && _is_initialized)
    {
        _A = A;
        _lA = lA;
        _uA = uA;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateBounds(const Vector &l, const Vector &u)
{
    if( l.size() == _l.size() && u.size() == _u.size() && _is_initialized)
    {
        _l = l;
        _u = u;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateProblem(const Matrix &H, const Vector &g,
                                   const Matrix &A, const Vector &lA, const Vector &uA,
                                   const Vector &l, const Vector &u)
{
    return updateTask(H, g) && updateConstraints(A, lA, uA) && updateBounds(l, u);
}

bool QPOasesProblem::addTask(const Matrix &H, const Vector &g, const bool init_problem)
{
    if(_is_initialized && H.cols() == _H.cols())
    {
        _H = pile(_H, H);
        _g = cat(_g, g);
        if(init_problem){
            _problem.reset();
            initProblem(_H, _g, _A, _lA, _uA, _l, _u);}
        return true;
    }
    return false;
}

bool QPOasesProblem::addConstraints(const Matrix &A, const Vector &lA, const Vector &uA,
                                    const bool init_problem)
{
    if(_is_initialized && A.cols() == _A.cols())
    {
        _A = pile(_A, A);
        _lA = cat(_lA, lA);
        _uA = cat(_uA, uA);
        if(init_problem){
            _problem.reset();
            initProblem(_H, _g, _A, _lA, _uA, _l, _u);}
        return true;
    }
    return false;
}

bool QPOasesProblem::addBounds(const Vector &l, const Vector &u, const bool init_problem)
{
    if(_is_initialized)
    {
        _l = cat(_l, l);
        _u = cat(_u, u);
        if(init_problem){
            _problem.reset();
            initProblem(_H, _g, _A, _lA, _uA, _l, _u);}
        return true;
    }
    return false;
}

bool QPOasesProblem::addProblem(const Matrix &H, const Vector &g,
                                const Matrix &A, const Vector &lA,
                                const Vector &uA, const Vector &l,
                                const Vector &u)
{
    return addTask(H, g) && addConstraints(A, lA, uA) && addBounds(l, u, true);
}

bool QPOasesProblem::solve()
{
    // To solve the problem it has to be initialized
    if(_is_initialized)
    {
        hack(); //<- PAY ATTENTION!

        int nWSR = _nWSR;
        _problem.hotstart(_H.data(),_g.data(),
                       _A_ptr,
                       _l_ptr, _u_ptr,
                       _lA_ptr,_uA_ptr,
                       nWSR,0);

        // If solution has changed of size we update the size
        if(_solution.size() != _problem.getNV())
            _solution.resize(_problem.getNV());

        if(_dual_solution.size() != _problem.getNV() + _problem.getNC())
            _dual_solution.resize(_problem.getNV()+ _problem.getNC());

        //We get the solution
        int success = _problem.getPrimalSolution(_solution.data());
        _problem.getDualSolution(_dual_solution.data());
        _problem.getBounds(_bounds);
        _problem.getConstraints(_constraints);

        if(success == qpOASES::RET_QP_NOT_SOLVED ||
          (success != qpOASES::RET_QP_SOLVED &&
           success != qpOASES::SUCCESSFUL_RETURN))
        {
            std::cout<<"ERROR OPTIMIZING TASK! ERROR "<<success<<std::endl;
            return false;
        }
        return true;
    }
    return false;
}

/// QPOasesTask ///
QPOasesTask::QPOasesTask(const boost::shared_ptr<Task<Matrix, Vector> > &task):
    QPOasesProblem(task->getXSize(),
                   wb_sot::bounds::velocity::Aggregated(task->getConstraints(), task->getXSize()).getAineq().rows(),
                   (qpOASES::HessianType)task->getHessianAtype()),

    _task(task)
{
    prepareData();
    assert(initProblem(_H, _g, _A, _lA, _uA, _l, _u));
}

QPOasesTask::~QPOasesTask()
{

}

void QPOasesTask::prepareData()
{
    /* Compute cost function */
    _H = _task->getA().transposed() * _task->getWeight() * _task->getA();
    _g = -1.0 * _task->getAlpha() * _task->getA().transposed() * _task->getWeight() * _task->getb();

    /* Compute constraints */
    using namespace  wb_sot::bounds::velocity;
    Aggregated constraints(_task->getConstraints(), _task->getXSize());
    _A = constraints.getAineq();
    _lA = constraints.getbLowerBound();
    _uA = constraints.getbUpperBound();

    /* Compute bounds */
    _l = constraints.getLowerBound();
    _u = constraints.getUpperBound();
}

bool QPOasesTask::solve()
{
    prepareData();
    return this->QPOasesProblem::solve();
}

/// QPOases_sot ///
QPOases_sot::QPOases_sot(vector<boost::shared_ptr<Task<Matrix, Vector> > > &stack_of_tasks):
_stack_of_tasks(stack_of_tasks)
{
    _qp_stack_of_tasks.reserve(_stack_of_tasks.size());
    assert(prepareSoT());
}

bool QPOases_sot::prepareSoT()
{
    for(unsigned int i = 0; i < _stack_of_tasks.size(); ++i)
    {
        _qp_stack_of_tasks.push_back(QPOasesTask(_stack_of_tasks[i]));
        if(i > 0)
        {
            _qp_stack_of_tasks.push_back(QPOasesTask(_stack_of_tasks[i]));
            expandProblem(i);
            //so cazzi...
        }

    }
}

bool QPOases_sot::expandProblem(unsigned int i)
{
    //I want to add all the constraints of the previous j tasks to task i
    for(unsigned int j = 0; j < i; ++j)
    {
        //1. Prepare new constraints from task j
            wb_sot::bounds::BilateralConstraint task_j_constraint
                    (_stack_of_tasks[j]->getA(),
                     _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[i]->getSolution(),
                     _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[i]->getSolution());
        //2. Get constraints & bounds of task j
            std::list< wb_sot::bounds::velocity::Aggregated::BoundType > constraints_list =
                _stack_of_tasks[j]->getConstraints().push_back(task_j_constraint);
            wb_sot::bounds::velocity::Aggregated
                new_constraints_for_task_i(constraints_list, _stack_of_tasks[j]->getXSize());
        //3. Add new constraints & bounds to problem i
            //3.1 ADD constraints to problem i

            //3.2 ADD bounds to problem i


    }
}

bool QPOases_sot::solve(Vector &solution)
{

}
