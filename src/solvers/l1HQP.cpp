#include <OpenSoT/solvers/l1HQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/Aggregated.h>
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
    OpenSoT::tasks::Aggregated::Ptr aggregated_task;
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> task_list;
    for(std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr>::iterator it = _lp_tasks.begin(); it != _lp_tasks.end(); it++)
        task_list.push_back(it->second);
    aggregated_task = boost::make_shared<OpenSoT::tasks::Aggregated>(task_list, _opt->getSize());
    _internal_stack = boost::make_shared<OpenSoT::AutoStack>(aggregated_task);
    _internal_stack->update(Eigen::VectorXd(0));
}

void l1HQP::creates_tasks()
{
    //1) here we creates the lp tasks
    std::map<std::string, Eigen::VectorXd>::iterator it;
    for(it = _linear_gains.begin(); it != _linear_gains.end(); it++){
        std::string task_id = it->first + "_task";
        _lp_tasks[it->first] = boost::make_shared<OpenSoT::tasks::GenericLPTask>(task_id, _linear_gains[it->first],
                _opt->getVariable(it->first));
    }

    //2) here we creates the other tasks which will became constraints

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
    return false;
}
