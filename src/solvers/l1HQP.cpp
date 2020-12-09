#include <OpenSoT/solvers/l1HQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <string>


using namespace OpenSoT::solvers;

l1HQP::l1HQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation,const solver_back_ends be_solver):
    Solver(stack_of_tasks.getStack(), stack_of_tasks.getBounds()),
    _epsRegularisation(eps_regularisation),
    _stack_of_tasks(stack_of_tasks)
{
    creates_problem_variables();
    creates_tasks();

    creates_internal_problem();
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
    if(_stack_of_tasks.getBounds())
        constraint_list.push_back(_stack_of_tasks.getBounds());



    _internal_stack = boost::make_shared<OpenSoT::AutoStack>(aggregated, constraint_list);


    _internal_stack->update(Eigen::VectorXd(0));
}

void l1HQP::creates_tasks()
{
    std::map<std::string, Eigen::VectorXd>::iterator it;

    //1) here we creates the lp tasks
    for(it = _linear_gains.begin(); it != _linear_gains.end(); it++){
        std::string task_id = it->first + "_task";
        _lp_tasks[it->first] = boost::make_shared<OpenSoT::tasks::GenericLPTask>(task_id, _linear_gains[it->first],
                _opt->getVariable(it->first));
    }

    //2) here we creates constraints from the given tasks
    unsigned int i = 0;
    for(it = _linear_gains.begin(); it != _linear_gains.end(); it++){
        std::string task_id = it->first + "_constraint";
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
    vars.emplace_back("x", problem_variables);
    XBot::Logger::info("Created x variable with size %i\n", problem_variables);

    int levels_of_priority = _stack_of_tasks.getStack().size();
    XBot::Logger::info("Problem has %i levels of priority\n", levels_of_priority);

    for(unsigned int i = 0; i < levels_of_priority; ++i)
    {
        int task_rows = _stack_of_tasks.getStack()[i]->getA().rows();
        XBot::Logger::info("Level %i task has %i rows\n", i, task_rows);
        std::string var_name = "t" + std::to_string(i);
        vars.emplace_back(var_name, task_rows);
        _linear_gains[var_name] = 10.*(levels_of_priority - i)*Eigen::VectorXd::Constant(task_rows, 1.);
        XBot::Logger::info("Created variable %s with size %i\n", var_name.c_str(), task_rows);
    }

    _opt = boost::make_shared<OptvarHelper>(vars);
}

bool l1HQP::solve(Eigen::VectorXd& solution)
{
    _internal_stack->update(Eigen::VectorXd(0));

    return false;
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
    _AA.pile(_task->getA());
    _AA.pile(-_task->getA());
    _AA.pile(O);

    _bb.pile(_task->getb());
    _bb.pile(-_task->getb());
    _bb.pile(o);

    _constraint = _AA.generate_and_get()*_x + _II.generate_and_get()*_t -_bb.generate_and_get();

    _Aineq = _constraint.getM();
    _bUpperBound = - _constraint.getq();

    _AA.reset();
    _bb.reset();
}
