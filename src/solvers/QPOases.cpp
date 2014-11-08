#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

using namespace yarp::math;

using namespace OpenSoT::solvers;

/// QPOasesProblem ///

QPOasesProblem::QPOasesProblem():
    _problem(new qpOASES::SQProblem()),
    _H(0,0), _g(0), _A(0,0), _lA(0), _uA(0), _l(0), _u(0),
    _bounds(new qpOASES::Bounds()),
    _constraints(new qpOASES::Constraints()),
    _nWSR(132),
    _epsRegularisation(2E2),
    _solution(0), _dual_solution(0),
    _is_initialized(false)
{

}

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
    _solution(number_of_variables), _dual_solution(number_of_variables),
    _is_initialized(false)
{
    setDefaultOptions();
}

QPOasesProblem::~QPOasesProblem()
{
}

void QPOasesProblem::setProblem(const boost::shared_ptr<qpOASES::SQProblem> &problem)
{
    _problem = problem;
}

void QPOasesProblem::setDefaultOptions()
{
    qpOASES::Options opt;
    opt.setToReliable();
    opt.printLevel = qpOASES::PL_LOW;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.epsRegularisation *= _epsRegularisation;
    _problem->setOptions(opt);
}

void QPOasesProblem::setOptions(const qpOASES::Options &options)
{
    _problem->setOptions(options);
}

qpOASES::Options QPOasesProblem::getOptions(){
    return _problem->getOptions();
}

bool QPOasesProblem::initProblem(const Matrix &H, const Vector &g,
                                 const Matrix &A,
                                 const Vector &lA, const Vector &uA,
                                 const Vector &l, const Vector &u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;

    assert(_l.size() == _u.size());

    if(!(_lA.size() == _A.rows())){
        std::cout<<"_lA size: "<<_lA.size()<<std::endl;
        std::cout<<"_A rows: "<<_A.rows()<<std::endl;
        assert(_lA.size() == _A.rows());}
    if(!(_lA.size() == _uA.size())){
        std::cout<<"_lA size: "<<_lA.size()<<std::endl;
        std::cout<<"_uA size: "<<_uA.size()<<std::endl;
        assert(_lA.size() == _uA.size());}

    int nWSR = _nWSR;
        qpOASES::returnValue val =_problem->init( _H.data(),_g.data(),
                       _A.data(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

    _is_initialized = true;

    if(val != qpOASES::SUCCESSFUL_RETURN)
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
    qpOASES::returnValue success = _problem->getPrimalSolution(_solution.data());
    _problem->getDualSolution(_dual_solution.data());
    _problem->getBounds(*_bounds);
    _problem->getConstraints(*_constraints);

    if(success != qpOASES::SUCCESSFUL_RETURN)
    {
        std::cout<<"ERROR GETTING PRIMAL SOLUTION IN INITIALIZATION! ERROR "<<success<<std::endl;
        _is_initialized = false;
    }
    return _is_initialized;
}

bool QPOasesProblem::updateTask(const Matrix &H, const Vector &g)
{
    assert(_H.rows() == H.rows());
    assert(_H.cols() == H.cols());
    assert(_g.size() == g.size());
    assert(_g.size() == _H.rows());

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
    assert(_A.cols() == _A.cols());

    if(!(_lA.size() == _A.rows())){
        std::cout<<"_lA size: "<<_lA.size()<<std::endl;
        std::cout<<"_A rows: "<<_A.rows()<<std::endl;
        assert(_lA.size() == _A.rows());}
    if(!(_lA.size() == _uA.size())){
        std::cout<<"_lA size: "<<_lA.size()<<std::endl;
        std::cout<<"_uA size: "<<_uA.size()<<std::endl;
        assert(_lA.size() == _uA.size());}

    if(_A.rows() != A.rows())
        _A.resize(A.rows(), A.cols());
    if(_lA.size() != lA.size())
        _lA.resize(lA.size());
    if(_uA.size() != uA.size())
        _uA.resize(uA.size());


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
    assert(l.size() == _l.size());
    assert(u.size() == _u.size());
    assert(l.size() == u.size());

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
        if(!(_lA.size() == _uA.size())){
            std::cout<<"_lA size: "<<_lA.size()<<std::endl;
            std::cout<<"_uA size: "<<_uA.size()<<std::endl;
            assert(_lA.size() == _uA.size());}
        if(!(_lA.size() == _A.rows())){
            std::cout<<"_lA size: "<<_lA.size()<<std::endl;
            std::cout<<"_A rows: "<<_A.rows()<<std::endl;
            assert(_lA.size() == _A.rows());}
        if(!(_l.size() == _u.size())){
            std::cout<<"_l size: "<<_l.size()<<std::endl;
            std::cout<<"_u size: "<<_u.size()<<std::endl;
            assert(_l.size() == _u.size());}

        int nWSR = _nWSR;
        qpOASES::returnValue val =_problem->hotstart(_H.data(),_g.data(),
                       _A.data(),
                       _l.data(), _u.data(),
                       _lA.data(),_uA.data(),
                       nWSR,0);

        if(val != qpOASES::SUCCESSFUL_RETURN)
        {
            std::cout<<"ERROR OPTIMIZING TASK! ERROR "<<val<<std::endl;
            //return false;
        }
        //else
        //{
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

            if(success != qpOASES::SUCCESSFUL_RETURN)
            {
                std::cout<<"ERROR GETTING PRIMAL SOLUTION! ERROR "<<success<<std::endl;
                //return false;
                resetProblem();
                return initProblem(_H, _g, _A, _lA, _uA, _l ,_u);
            }
            return true;
        //}
    }
    return false;
}

OpenSoT::HessianType QPOasesProblem::getHessianType() {return (OpenSoT::HessianType)(_problem->getHessianType());}

void QPOasesProblem::setHessianType(const OpenSoT::HessianType ht){_problem->setHessianType((qpOASES::HessianType)(ht));}

bool QPOasesProblem::resetProblem(){return _problem->reset();}

/// QPOasesTask ///
QPOasesTask::QPOasesTask(const boost::shared_ptr<Task<Matrix, Vector> > &task, const double eps_regularisation):
    QPOasesProblem(task->getXSize(),
                   OpenSoT::constraints::Aggregated(task->getConstraints(), task->getXSize()).getAineq().rows(),
                   (OpenSoT::HessianType)(task->getHessianAtype()), eps_regularisation),
    _task(task)
{
    prepareData(true);
    printProblemInformation(-1);
    assert(initProblem(_H, _g, _A, _lA, _uA, _l, _u));
}

QPOasesTask::~QPOasesTask()
{

}

void QPOasesTask::prepareData(bool update_constraints)
{
    /* Set Hessian Type */
    //setHessianType(_task->getHessianAtype());

    /* Compute cost function */
    _H = _task->getA().transposed() * _task->getWeight() * _task->getA();
    _g = -1.0 * _task->getLambda() * _task->getA().transposed() * _task->getWeight() * _task->getb();

    if(update_constraints)
    {
        /* Compute constraints */
        using namespace  OpenSoT::constraints;
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
    std::cout<<GREEN<<"eps Regularisation factor: "<<DEFAULT<<getOptions().epsRegularisation<<std::endl;
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
QPOases_sot::QPOases_sot(Stack &stack_of_tasks, const double eps_regularisation):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation)
{
    _qp_stack_of_tasks.reserve(_tasks.size());
    assert(prepareSoT());
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         boost::shared_ptr<constraints::Aggregated> &bounds, const double eps_regularisation):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
    _qp_stack_of_tasks.reserve(_tasks.size());
    assert(prepareSoT());
}

bool QPOases_sot::prepareSoT()
{
    bool prepared = true;
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        _qp_stack_of_tasks.push_back(QPOasesTask(_tasks[i], _epsRegularisation));
        if(i > 0)
        {
            prepared = prepared && expandProblem(i);
        }
        else //add to first task only bounds
        {
            yarp::sig::Vector li, ui;
            _qp_stack_of_tasks[i].getBounds(li, ui);
            assert(li.size() == 0);
            assert(ui.size() == 0);
            if(_bounds)
                prepared = prepared && _qp_stack_of_tasks[i].addBounds(
                                _bounds->getLowerBound(),
                                _bounds->getUpperBound());
        }
        if(prepared)
            _qp_stack_of_tasks[i].printProblemInformation(i);
        else
            std::cout<<"ERROR Occurered preparing task "<<_qp_stack_of_tasks[i].getTaskID()<<std::endl;
    }
    return prepared;
}

bool QPOases_sot::expandProblem(unsigned int i)
{
    bool constraints_added = false;
    bool bounds_added = true;

    //I want to add all the constraints of the previous j tasks to task i
    for(unsigned int j = 0; j < i; ++j)
    {
        //1. Prepare new constraints from task j
            boost::shared_ptr<OpenSoT::constraints::BilateralConstraint> task_j_constraint(
                new OpenSoT::constraints::BilateralConstraint
                    (
                        _tasks[j]->getA(),
                        _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                        _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                     )
            );
        //2. Get constraints of task j
            std::list< boost::shared_ptr< OpenSoT::constraints::Aggregated::ConstraintType > > constraints_list =
                _tasks[j]->getConstraints();
            constraints_list.push_back(task_j_constraint);
            OpenSoT::constraints::Aggregated
                new_constraints_for_task_i(constraints_list, _tasks[j]->getXSize());

        //3. Add new constraints & bounds to problem i
            //3.1 ADD constraints to problem i
                constraints_added = _qp_stack_of_tasks[i].addConstraints(
                            new_constraints_for_task_i.getAineq(),
                            new_constraints_for_task_i.getbLowerBound(),
                            new_constraints_for_task_i.getbUpperBound());
    }
        //4 ADD bounds to problem i
        yarp::sig::Vector li, ui;
        _qp_stack_of_tasks[i].getBounds(li, ui);
        assert(li.size() == 0);
        assert(ui.size() == 0);
        if(_bounds)
            bounds_added = _qp_stack_of_tasks[i].addBounds(
                            _bounds->getLowerBound(),
                            _bounds->getUpperBound());

    return constraints_added && bounds_added;
}

unsigned int QPOases_sot::getNumberOfTasks()
{
    if(!(_qp_stack_of_tasks.size() == _tasks.size()))
    {
        std::cout<<"Stored # QP Problems does not mach stored # tasks"<<std::endl;
        std::cout<<"# TASKS: "<<_tasks.size()<<std::endl;
        std::cout<<"# QP PROBLEMS: "<<_qp_stack_of_tasks.size()<<std::endl;
        assert(_qp_stack_of_tasks.size() == _tasks.size());
    }

    return _qp_stack_of_tasks.size();
}

bool QPOases_sot::solve(Vector &solution)
{
    bool solved_task_i = true;
    bool expanded = true;
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        expanded = expanded && updateExpandedProblem(i);
        bool a = _qp_stack_of_tasks[i].solve(false);
        if(!a) std::cout<<"TASK "<<i<<std::endl;
        solved_task_i =  solved_task_i && a;
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
    OpenSoT::constraints::Aggregated
        constraints_task_i(_tasks[i]->getConstraints(),
                           _tasks[i]->getXSize());

    yarp::sig::Matrix A = constraints_task_i.getAineq();
    yarp::sig::Vector lA = constraints_task_i.getbLowerBound();
    yarp::sig::Vector uA = constraints_task_i.getbUpperBound();

    //2. I got all the updated constraints from the previous tasks
    for(unsigned int j = 0; j < i; ++j)
    {
        //2.1 Prepare new constraints from task j
        boost::shared_ptr<OpenSoT::constraints::BilateralConstraint> task_j_constraint(
            new OpenSoT::constraints::BilateralConstraint
                (
                    _tasks[j]->getA(),
                    _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution(),
                    _tasks[j]->getA()*_qp_stack_of_tasks[j].getSolution()
                 )
        );
        //2.2 Get constraints & bounds of task j
        std::list< boost::shared_ptr< OpenSoT::constraints::Aggregated::ConstraintType > > constraints_list =
            _tasks[j]->getConstraints();
        constraints_list.push_back(task_j_constraint);
        OpenSoT::constraints::Aggregated
            updated_constraints_for_task_i(constraints_list,
                                           _tasks[j]->getXSize());

        //3. Stack updated constraints to constraints in task i
        A = yarp::math::pile(A, updated_constraints_for_task_i.getAineq());
        lA = yarp::math::cat(lA, updated_constraints_for_task_i.getbLowerBound());
        uA = yarp::math::cat(uA, updated_constraints_for_task_i.getbUpperBound());
   }
    //3. Update constraints matrices of task j
    ///TO DO: check that matrices from point 2. and matrices in _qp_stack_of_tasks[i]
    /// has the same size!
   bool problem_updated = _qp_stack_of_tasks[i].updateConstraints(A, lA, uA);
   if(_bounds)
        problem_updated = problem_updated && _qp_stack_of_tasks[i].updateBounds(_bounds->getLowerBound(),
                                              _bounds->getUpperBound());
    return problem_updated;
}





bool QPOases_sot::setOptions(const unsigned int i, const qpOASES::Options &opt)
{
    if(i > _qp_stack_of_tasks.size())
    {
        std::cout<<"Index out of range!"<<std::endl;
        return false;
    }

    _qp_stack_of_tasks[i].setOptions(opt);
    return true;
}

bool QPOases_sot::getOptions(const unsigned int i, qpOASES::Options& opt)
{

    if(i > _qp_stack_of_tasks.size())
    {
        std::cout<<"Index out of range!"<<std::endl;
        return false;
    }

    opt = _qp_stack_of_tasks[i].getOptions();
    return true;
}
