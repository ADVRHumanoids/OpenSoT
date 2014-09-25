#include <wb_sot/solvers/QPOases.h>
#include <yarp/math/Math.h>
#include <wb_sot/bounds/BilateralConstraint.h>
#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

using namespace yarp::math;

using namespace wb_sot::solvers;

/// QPOasesProblem ///

QPOasesProblem::QPOasesProblem():
    _problem(new qpOASES::SQProblem()),
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
    _problem(new qpOASES::SQProblem(number_of_variables, number_of_constraints, hessian_type)),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(),
    _constraints(),
    _nWSR(132),
    _solution(number_of_variables), _dual_solution(number_of_variables),
    _is_initialized(false)
{
    setDefaultOptions();
}

void QPOasesProblem::setProblem(const boost::shared_ptr<qpOASES::SQProblem> &problem)
{
    _problem = problem;
}

void QPOasesProblem::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToReliable();
    opt.printLevel = qpOASES::PL_NONE;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= 2E2;
    _problem->setOptions(opt);
}

void QPOasesProblem::setOptions(const qpOASES::Options &options)
{
    _problem->setOptions(options);
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
        qpOASES::returnValue val =_problem->init( _H.data(),_g.data(),
                       _A_ptr,
                       _l_ptr, _u_ptr,
                       _lA_ptr,_uA_ptr,
                       nWSR,0);

    _is_initialized = true;

    if(!(val == qpOASES::SUCCESSFUL_RETURN))
    {
        _is_initialized = false;
        std::cout<<"ERROR INITIALIZING QP PROBLEM "<<std::endl;
        std::cout<<"CODE ERROR: "<<val<<std::endl;
        return _is_initialized;
    }

    if(_solution.size() != _problem->getNV())
        _solution.resize(_problem->getNV());

    if(_dual_solution.size() != _problem->getNV() + _problem->getNC())
        _dual_solution.resize(_problem->getNV() + _problem->getNC());

    //We get the solution
    int success = _problem->getPrimalSolution(_solution.data());
    _problem->getDualSolution(_dual_solution.data());
    _problem->getBounds(_bounds);
    _problem->getConstraints(_constraints);

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
    if(_is_initialized)
    {
        _H = H;
        _g = g;
        return true;
    }
    return false;
}

bool QPOasesProblem::updateConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if(_is_initialized)
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
    if(_is_initialized)
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

bool QPOasesProblem::addTask(const Matrix &H, const Vector &g)
{
    if(_is_initialized && H.cols() == _H.cols())
    {
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
    return false;
}

bool QPOasesProblem::addConstraints(const Matrix &A, const Vector &lA, const Vector &uA)
{
    if(_is_initialized && A.cols() == _A.cols())
    {
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
    return false;
}

bool QPOasesProblem::addBounds(const Vector &l, const Vector &u)
{
    if(_is_initialized)
    {
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
    return false;
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
    // To solve the problem it has to be initialized
    if(_is_initialized)
    {
        hack(); //<- PAY ATTENTION!

        int nWSR = _nWSR;
        _problem->hotstart(_H.data(),_g.data(),
                       _A_ptr,
                       _l_ptr, _u_ptr,
                       _lA_ptr,_uA_ptr,
                       nWSR,0);

        // If solution has changed of size we update the size
        if(_solution.size() != _problem->getNV())
            _solution.resize(_problem->getNV());

        if(_dual_solution.size() != _problem->getNV() + _problem->getNC())
            _dual_solution.resize(_problem->getNV()+ _problem->getNC());

        //We get the solution
        int success = _problem->getPrimalSolution(_solution.data());
        _problem->getDualSolution(_dual_solution.data());
        _problem->getBounds(_bounds);
        _problem->getConstraints(_constraints);

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
                   wb_sot::bounds::Aggregated(task->getConstraints(), task->getXSize()).getAineq().rows(),
                   (qpOASES::HessianType)task->getHessianAtype()),

    _task(task)
{
    prepareData();
    printProblemInformation(-1);
    assert(initProblem(_H, _g, _A, _lA, _uA, _l, _u));
}

QPOasesTask::~QPOasesTask()
{

}

void QPOasesTask::prepareData(bool update_constraints)
{
    /* Compute cost function */
    _H = _task->getA().transposed() * _task->getWeight() * _task->getA();
    _g = -1.0 * _task->getAlpha() * _task->getA().transposed() * _task->getWeight() * _task->getb();

    if(update_constraints)
    {
        /* Compute constraints */
        using namespace  wb_sot::bounds;
        Aggregated constraints(_task->getConstraints(), _task->getXSize());
        _A = constraints.getAineq();
        _lA = constraints.getbLowerBound();
        _uA = constraints.getbUpperBound();

        /* Compute bounds */
        _l = constraints.getLowerBound();
        _u = constraints.getUpperBound();
    }
}

bool QPOasesTask::solve(bool update_constraints)
{
    prepareData(update_constraints);
    return this->QPOasesProblem::solve();
}

void QPOasesTask::printProblemInformation(int i)
{
    std::cout<<std::endl;
    if(i == -1)
        std::cout<<GREEN<<"PROBLEM ID: "<<DEFAULT<<_task->getTaskID()<<std::endl;
    else
        std::cout<<GREEN<<"PROBLEM "<<i<<" ID: "<<DEFAULT<<_task->getTaskID()<<std::endl;
    std::cout<<GREEN<<"# OF CONSTRAINTS: "<<DEFAULT<<_lA.size()<<std::endl;
    std::cout<<GREEN<<"# OF BOUNDS: "<<DEFAULT<<_l.size()<<std::endl;
    std::cout<<GREEN<<"# OF VARIABLES: "<<DEFAULT<<_task->getXSize()<<std::endl;
//    std::cout<<GREEN<<"H: "<<DEFAULT<<_H.toString()<<std::endl;
//    std::cout<<GREEN<<"g: "<<DEFAULT<<_g.toString()<<std::endl;
//    std::cout<<GREEN<<"A: "<<DEFAULT<<_A.toString()<<std::endl;
//    std::cout<<GREEN<<"lA: "<<DEFAULT<<_lA.toString()<<std::endl;
//    std::cout<<GREEN<<"uA: "<<DEFAULT<<_uA.toString()<<std::endl;
//    std::cout<<GREEN<<"u: "<<DEFAULT<<_u.toString()<<std::endl;
//    std::cout<<GREEN<<"l: "<<DEFAULT<<_l.toString()<<std::endl;
    std::cout<<std::endl;
}

void QPOasesTask::getCostFunction(Matrix &H, Vector &g)
{
    H = _H;
    g = _g;
}

void QPOasesTask::getConstraints(Matrix &A, Vector &lA, Vector &uA)
{
    A = _A;
    lA = _lA;
    uA = _uA;
}

void QPOasesTask::getBounds(Vector &l, Vector &u)
{
    l = _l;
    u = _u;
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
            expandProblem(i);
        }
        _qp_stack_of_tasks[i].printProblemInformation(i);
    }
    return true;
}

bool QPOases_sot::expandProblem(unsigned int i)
{
    bool constraints_added = false;
    bool bounds_added = true;

    //I want to add all the constraints of the previous j tasks to task i
    for(unsigned int j = 0; j < i; ++j)
    {
        //1. Prepare new constraints from task j
            boost::shared_ptr<wb_sot::bounds::BilateralConstraint> task_j_constraint(
                new wb_sot::bounds::BilateralConstraint
                    (
                        _stack_of_tasks[j]->getA(),
                        _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                        _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                     )
            );
        //2. Get constraints & bounds of task j
            std::list< boost::shared_ptr< wb_sot::bounds::Aggregated::BoundType > > constraints_list =
                _stack_of_tasks[j]->getConstraints();
            constraints_list.push_back(task_j_constraint);
            wb_sot::bounds::Aggregated
                new_constraints_for_task_i(constraints_list, _stack_of_tasks[j]->getXSize());
        //3. Add new constraints & bounds to problem i
            //3.1 ADD constraints to problem i
                constraints_added = _qp_stack_of_tasks[i].addConstraints(
                            new_constraints_for_task_i.getAineq(),
                            new_constraints_for_task_i.getbLowerBound(),
                            new_constraints_for_task_i.getbUpperBound());
            //3.2 ADD bounds to problem i
                yarp::sig::Vector li, ui;
                _qp_stack_of_tasks[i].getBounds(li, ui);
                if(li.size() == 0)
                    bounds_added = _qp_stack_of_tasks[i].addBounds(
                                new_constraints_for_task_i.getLowerBound(),
                                new_constraints_for_task_i.getUpperBound());
                else
                {
                    for(unsigned int k = 0; k < li.size(); ++k)
                    {
                        li[k] = std::max(li[k], new_constraints_for_task_i.getLowerBound()[k]);
                        ui[k] = std::min(ui[k], new_constraints_for_task_i.getUpperBound()[k]);
                    }
                    _qp_stack_of_tasks[i].updateBounds(li, ui);
                }
    }
    return constraints_added && bounds_added;
}

bool QPOases_sot::solve(Vector &solution)
{
    bool solved_task_i = false;
    bool expanded = true;
    for(unsigned int i = 0; i < _stack_of_tasks.size(); ++i)
    {
        bool update_constraints = true;
        if(i > 0)
        {
            expanded = updateExpandedProblem(i);
            update_constraints = false;
        }
        solved_task_i =  _qp_stack_of_tasks[i].solve(update_constraints);
        solution = _qp_stack_of_tasks[i].getSolution();

        //_qp_stack_of_tasks[i].printProblemInformation(i);
        //std::cout<<"SOLUTION PROBLEM i: "<<solution.toString()<<std::endl;
    }
    return solved_task_i && expanded;
}

bool QPOases_sot::updateExpandedProblem(unsigned int i)
{
    //I want to update all the constraints of the previous j tasks to task i
    //1. I got all the updated constraints from task i
    wb_sot::bounds::Aggregated
        constraints_task_i(_stack_of_tasks[i]->getConstraints(),
                           _stack_of_tasks[i]->getXSize());

    yarp::sig::Matrix A = constraints_task_i.getAineq();
    yarp::sig::Vector lA = constraints_task_i.getbLowerBound();
    yarp::sig::Vector uA = constraints_task_i.getbUpperBound();
    yarp::sig::Vector l = constraints_task_i.getLowerBound();
    yarp::sig::Vector u = constraints_task_i.getUpperBound();

    //2. I got all the updated constraints from the previous tasks
    for(unsigned int j = 0; j < i; ++j)
    {
        //2.1 Prepare new constraints from task j
        boost::shared_ptr<wb_sot::bounds::BilateralConstraint> task_j_constraint(
            new wb_sot::bounds::BilateralConstraint
                (
                    _stack_of_tasks[j]->getA(),
                    _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                    _stack_of_tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                 )
        );
        //2.2 Get constraints & bounds of task j
        std::list< boost::shared_ptr< wb_sot::bounds::Aggregated::BoundType > > constraints_list =
            _stack_of_tasks[j]->getConstraints();
        constraints_list.push_back(task_j_constraint);
        wb_sot::bounds::Aggregated
            updated_constraints_for_task_i(constraints_list,
                                           _stack_of_tasks[j]->getXSize());

        //3. Stack updated constraints to constraints in task i
        A = yarp::math::pile(A, updated_constraints_for_task_i.getAineq());
        lA = yarp::math::cat(lA, updated_constraints_for_task_i.getbLowerBound());
        uA = yarp::math::cat(uA, updated_constraints_for_task_i.getbUpperBound());
        if(l.size() == 0)
        {
            l = updated_constraints_for_task_i.getLowerBound();
            u = updated_constraints_for_task_i.getUpperBound();
        }
        else
        {
            for(unsigned int i = 0; i < l.size(); ++i)
            {
                l[i] = std::max(l[i], updated_constraints_for_task_i.getLowerBound()[i]);
                u[i] = std::min(u[i], updated_constraints_for_task_i.getUpperBound()[i]);
            }
        }
   }
    //3. Update constraints matrices of task j
    ///TO DO: check that matrices from point 2. and matrices in _qp_stack_of_tasks[i]
    /// has the same size!
   return _qp_stack_of_tasks[i].updateConstraints(A, lA, uA) &&
           _qp_stack_of_tasks[i].updateBounds(l, u);
}

std::vector<std::pair<std::string, int>> QPOases_sot::getNumberOfConstraintsInQP()
{
    std::vector<std::pair<std::string, int>> v;
    std::pair<std::string, int> a;

    for(unsigned int i = 0; i < getNumberOfTasks(); ++i)
    {
        a.first = _stack_of_tasks[i]->getTaskID();
        a.second = 0;
        yarp::sig::Vector l,u,lA, uA;
        yarp::sig::Matrix A;
        _qp_stack_of_tasks[i].getConstraints(A, lA, uA);
        _qp_stack_of_tasks[i].getBounds(l, u);
        if(lA.size() > 0)
            a.second = lA.size();
        if(l.size() > 0)
            a.second += l.size();
        v.push_back(a);
    }
    return v;
}

std::vector<std::pair<std::string, int>> QPOases_sot::getNumberOfConstraintsInTaskList()
{
    std::vector<std::pair<std::string, int>> v;
    std::pair<std::string, int> a;

    for(unsigned int i = 0; i < getNumberOfTasks(); ++i)
    {
        a.first = _stack_of_tasks[i]->getTaskID();
        a.second = 0;

        wb_sot::bounds::Aggregated
            constraints_task_i(_stack_of_tasks[i]->getConstraints(),
                                           _stack_of_tasks[i]->getXSize());
        yarp::sig::Vector lA = constraints_task_i.getbLowerBound();
        yarp::sig::Vector l = constraints_task_i.getLowerBound();
        if(lA.size() > 0)
            a.second = lA.size();
        if(l.size() > 0)
            a.second += l.size();
        v.push_back(a);
    }
    return v;
}
