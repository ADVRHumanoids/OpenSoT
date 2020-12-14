#include <OpenSoT/solvers/l1HQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <string>
#include <math.h>

#define ENABLE_PRIORITY_CONSTRAINT true

using namespace OpenSoT::solvers;

l1HQP::l1HQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation,const solver_back_ends be_solver):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds()),
    _epsRegularisation(eps_regularisation),
    _stack_of_tasks(stack_of_tasks)
{    
    if(std::fpclassify(eps_regularisation) == FP_ZERO) //No L2-regularisation
        _hessian_type = HessianType::HST_ZERO;
    else
    {
        _hessian_type = HessianType::HST_IDENTITY;
        XBot::Logger::info("Adding L2 regularisation \n");
    }

    creates_problem_variables();
    _H.setZero(_opt->getSize(), _opt->getSize());
    creates_tasks();
    creates_constraints();
    creates_internal_problem();
    if(!creates_solver(be_solver))
        throw std::runtime_error("Can not initialize internal solver!");
}

bool l1HQP::creates_solver(const solver_back_ends solver_back_end)
{
    _solver = BackEndFactory(solver_back_end,
                   _internal_stack->getStack()[0]->getXSize(),
                   _internal_stack->getBounds()->getAineq().rows(),
                   _hessian_type, _epsRegularisation);

    bool success = _solver->initProblem(_H,
                   _internal_stack->getStack()[0]->getc().transpose(),
                   _internal_stack->getBounds()->getAineq(),
                   _internal_stack->getBounds()->getbLowerBound(), _internal_stack->getBounds()->getbUpperBound(),
                   Eigen::VectorXd(0), Eigen::VectorXd(0));

    if(success)
        _internal_solution = _solver->getSolution();

    return success;
}

void l1HQP::creates_internal_problem()
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> task_list;
    for(std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr>::iterator it = _lp_tasks.begin(); it != _lp_tasks.end();
        it++)
        task_list.push_back(it->second);
    OpenSoT::tasks::Aggregated::Ptr aggregated =
            boost::make_shared<OpenSoT::tasks::Aggregated>(task_list, _opt->getSize());



    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> constraint_list;
    for(std::map<std::string, task_to_constraint_helper::Ptr>::iterator it = _constraints.begin(); it != _constraints.end();
        it++)
        constraint_list.push_back(it->second);


    if(_constraints2)
        constraint_list.push_back(_constraints2);

#if ENABLE_PRIORITY_CONSTRAINT
    for(unsigned int i = 0; i < _priority_constraints.size(); ++i)
        constraint_list.push_back(_priority_constraints[i]);
#endif

    _internal_stack = boost::make_shared<OpenSoT::AutoStack>(aggregated, constraint_list);


    _internal_stack->update(Eigen::VectorXd(0));
}

void l1HQP::creates_constraints()
{
#if ENABLE_PRIORITY_CONSTRAINT
    //1. Adding constraints to enforce priorities if present
    if(_task_id_priority_order.size() > 1)
    {
        std::string high_priority_task = "";
        std::string low_priority_task = "";
        std::string id = "";
        for(unsigned int i = 0; i < _task_id_priority_order.size()-1; ++i)
        {
            high_priority_task = _task_id_priority_order[i];
            low_priority_task = _task_id_priority_order[i+1];

            id = "t"+std::to_string(i)+"_"+"t"+std::to_string(i+1)+"_constr";
            _priority_constraints.push_back(boost::make_shared<priority_constraint>(id,
                                        _lp_tasks[high_priority_task],
                                        _lp_tasks[low_priority_task]));
        }
    }
#endif


    //2. Constraints coming from the original problem are kept
    if(_stack_of_tasks.getBounds())
        _constraints2 = boost::make_shared<constraint_helper>("internal_constraints", _stack_of_tasks.getBounds(),
                                                              _opt->getVariable("x"));
}

void l1HQP::creates_tasks()
{
    std::map<std::string, Eigen::VectorXd>::iterator it;

    //1) here we creates the lp tasks
    std::string task_id = "";
    for(it = _linear_gains.begin(); it != _linear_gains.end(); it++){
        task_id = it->first + "_task";
        _lp_tasks[it->first] = boost::make_shared<OpenSoT::tasks::GenericLPTask>(task_id, _linear_gains[it->first],
                _opt->getVariable(it->first));
    }

    //2) here we creates constraints from the given tasks
    unsigned int i = 0;
    for(it = _linear_gains.begin(); it != _linear_gains.end(); it++){
        task_id = it->first + "_constraint";
        _constraints[it->first] = boost::make_shared<task_to_constraint_helper>(task_id,
                _stack_of_tasks.getStack()[i],
                _opt->getVariable("x"),
                _opt->getVariable(it->first));
        i += 1;
    }
}

void l1HQP::creates_problem_variables()
{
    OptvarHelper::VariableVector vars;
    int problem_variables = _stack_of_tasks.getStack()[0]->getA().cols();
    XBot::Logger::info("Problem has %i variables\n", problem_variables);
    vars.emplace_back("x", problem_variables); //these are the variables of the original problem
    XBot::Logger::info("Created x variable with size %i\n", problem_variables);

    int levels_of_priority = _stack_of_tasks.getStack().size();
    XBot::Logger::info("Problem has %i levels of priority\n", levels_of_priority);

    std::string var_name = "";
    for(unsigned int i = 0; i < levels_of_priority; ++i)
    {
        int task_rows = _stack_of_tasks.getStack()[i]->getA().rows();
        XBot::Logger::info("Level %i task has %i rows\n", i, task_rows);
        var_name = "t" + std::to_string(i);
        vars.emplace_back(var_name, task_rows); //Slack variables associated to tasks
        int alpha = (levels_of_priority - i);
        _linear_gains[var_name] = std::pow(10., alpha)*alpha*Eigen::VectorXd::Constant(task_rows, 1.);
        XBot::Logger::info("Created variable %s with size %i\n", var_name.c_str(), task_rows);
        _task_id_priority_order.push_back(var_name);
    }

    _opt = boost::make_shared<OptvarHelper>(vars);
}

bool l1HQP::solve(Eigen::VectorXd& solution)
{   
    _internal_stack->update(Eigen::VectorXd(0));

    if(!_solver->updateProblem(_H, _internal_stack->getStack()[0]->getc().transpose(),
        _internal_stack->getBounds()->getAineq(),
        _internal_stack->getBounds()->getbLowerBound(), _internal_stack->getBounds()->getbUpperBound(),
        Eigen::VectorXd(0), Eigen::VectorXd(0)))
        return false;

    if(!_solver->solve())
        return false;

    _internal_solution = _solver->getSolution();

    _opt->getVariable("x").getValue(_internal_solution, solution);

    return true;
}

bool l1HQP::getInternalVariable(const std::string& var, Eigen::VectorXd& value)
{
    try{
        _opt->getVariable(var);
    } catch (const std::invalid_argument& e) {
        return false;
    }

    if(_internal_solution.size() > 0)
        _opt->getVariable(var).getValue(_internal_solution, value);
    else
        return false;

    return true;
}

task_to_constraint_helper::task_to_constraint_helper(std::string id, OpenSoT::tasks::Aggregated::TaskPtr& task,
           const AffineHelper& x, const AffineHelper& t):
    OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >(id, x.getInputSize()),
    _task(task), _x(x), _t(t), _II(task->getA().rows()), _AA(_task->getA().cols()), _bb(1)
{
    Eigen::MatrixXd I;
    I.setIdentity(task->getA().rows(),task->getA().rows());
    _II.pile(-I);
    _II.pile(-I);
    _II.pile(-I);

    _bLowerBound.resize(3*task->getA().rows());
    _bLowerBound = -1.0e20*_bLowerBound.setOnes(_bLowerBound.size());

    o.setZero(_task->getb().size());
    O.setZero(task->getA().rows(),task->getA().cols());

    update(Eigen::VectorXd(0));
}

void task_to_constraint_helper::update(const Eigen::VectorXd& x)
{
    _AA.pile(_task->getWA());
    _AA.pile(-_task->getWA());
    _AA.pile(O);

    _bb.pile(_task->getWb());
    _bb.pile(-_task->getWb());
    _bb.pile(o);

    _constraint = _AA.generate_and_get()*_x + _II.generate_and_get()*_t -_bb.generate_and_get();

    _Aineq = _constraint.getM();
    _bUpperBound = - _constraint.getq();

    _AA.reset();
    _bb.reset();
}

constraint_helper::constraint_helper(std::string id, OpenSoT::constraints::Aggregated::ConstraintPtr constraints,
                                     const AffineHelper& x):
    OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >(id, x.getInputSize()),
    _constraints(constraints), _x(x), _A(x.getInputSize()), _b_lower(1), _b_upper(1)
{
    if(_constraints->getLowerBound().size() > 0)
        I.setIdentity(_constraints->getLowerBound().size(), _constraints->getLowerBound().size());

    update(Eigen::VectorXd(0));
}

void constraint_helper::update(const Eigen::VectorXd& x)
{


    if(_constraints->getAineq().rows() > 0)
    {
        _constraint = _constraints->getAineq()*_x;
        _A.pile(_constraint.getM());
        _b_lower.pile(_constraints->getbLowerBound());
        _b_upper.pile(_constraints->getbUpperBound());
    }

    if(_constraints->getAeq().rows() > 0 ) //TODO: equality constraints
    {

    }

    if(_constraints->getLowerBound().size() > 0) //bounds
    {
        _constraint = I*_x;
        _A.pile(_constraint.getM());
        _b_lower.pile(_constraints->getLowerBound());
        _b_upper.pile(_constraints->getUpperBound());
    }

    _Aineq = _A.generate_and_get();
    _bLowerBound = _b_lower.generate_and_get();
    _bUpperBound = _b_upper.generate_and_get();

    _A.reset();
    _b_lower.reset();
    _b_upper.reset();
}

priority_constraint::priority_constraint(const std::string& id,
                                         const OpenSoT::tasks::GenericLPTask::Ptr high_priority_task,
                                         const OpenSoT::tasks::GenericLPTask::Ptr low_priority_task):
    OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd >(id, high_priority_task->getXSize()),
    _high_task(high_priority_task),
    _low_task(low_priority_task)
{
    _bLowerBound.setConstant(1, -1.0e20);
    _bUpperBound.setConstant(1, 0.);

    _Aineq.resize(1, _high_task.lock()->getc().size());
    _ones = (_high_task.lock()->getc().array() != 0).matrix().cast<double>() - (_low_task.lock()->getc().array() != 0).matrix().cast<double>();

    _Aineq = _ones.transpose();
}

void priority_constraint::update(const Eigen::VectorXd &x)
{

}
