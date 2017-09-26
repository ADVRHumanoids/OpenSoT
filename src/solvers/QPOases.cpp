#include <qpOASES.hpp>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/BilateralConstraint.h>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::solvers;

QPOases_sot::QPOases_sot(Stack &stack_of_tasks, const double eps_regularisation):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT())
        throw "Can Not initizalize SoT!";
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         ConstraintPtr bounds,
                         const double eps_regularisation):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT())
        throw "Can Not initizalize SoT with bounds!";
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         ConstraintPtr bounds,
                         ConstraintPtr globalConstraints,
                         const double eps_regularisation):
    Solver(stack_of_tasks, bounds, globalConstraints),
    _epsRegularisation(eps_regularisation)
{
    for(unsigned int i = 0; i < stack_of_tasks.size(); ++i)
        _active_stacks.push_back(true);

    if(!prepareSoT())
        throw "Can Not initizalize SoT with bounds!";
}

void QPOases_sot::computeCostFunction(const TaskPtr& task, Eigen::MatrixXd& H, Eigen::VectorXd& g)
{
//    H = task->getA().transpose() * task->getWeight() * task->getA();
//    g = -1.0 * task->getA().transpose() * task->getWeight() * task->getb();

    if(task->getWeight().isIdentity())
    {
        H.noalias() = task->getATranspose() * task->getA();
        g.noalias() = -1.0 * task->getATranspose() * task->getb();
    }
    else
    {
        H.noalias() = task->getATranspose() * task->getWA();
        g.noalias() = -1.0 * task->getATranspose() * task->getWb();
    }
}

void QPOases_sot::computeOptimalityConstraint(  const TaskPtr& task, QPOasesProblem& problem,
                                                Matrix& A, Vector& lA, Vector& uA)
{
    A = task->getA();
    lA = task->getA()*problem.getSolution();
    uA = lA;
}

bool QPOases_sot::prepareSoT()
{
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        computeCostFunction(_tasks[i], H, g);

        OpenSoT::constraints::Aggregated constraints_task_i(_tasks[i]->getConstraints(), _tasks[i]->getXSize());
        if(_globalConstraints){
            constraints_task_i.getConstraintsList().push_back(_globalConstraints);
            constraints_task_i.generateAll();}
        else if(_bounds && _bounds->isConstraint())
        {
            constraints_task_i.getConstraintsList().push_back(_bounds);
            constraints_task_i.generateAll();
        }

        std::string constraints_str = constraints_task_i.getConstraintID();

        A = constraints_task_i.getAineq();
        lA = constraints_task_i.getbLowerBound();
        uA = constraints_task_i.getbUpperBound();
        if(i > 0)
        {
            for(unsigned int j = 0; j < i; ++j)
            {
                computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A, tmp_lA, tmp_uA);

                if(!constraints_str.compare("") == 0)
                    constraints_str = constraints_str + "+";
                constraints_str = constraints_str + _tasks[j]->getTaskID() + "_optimality";

                pile(A, tmp_A);
                pile(lA, tmp_lA);
                pile(uA, tmp_uA);
            }
        }

        if(_bounds && _bounds->isBound()){   // if it is a constraint, it has already been added in #74
            constraints_task_i.getConstraintsList().push_back(_bounds);
            constraints_task_i.generateAll();}
        l = constraints_task_i.getLowerBound();
        u = constraints_task_i.getUpperBound();

        QPOasesProblem problem_i(_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
                                 _epsRegularisation);

        if(problem_i.initProblem(H, g, A, lA, uA, l, u)){
            _qp_stack_of_tasks.push_back(problem_i);
            std::string bounds_string = "";
            if(_bounds)
                bounds_string = _bounds->getConstraintID();
            _qp_stack_of_tasks[i].printProblemInformation(i, _tasks[i]->getTaskID(),
                                                          constraints_str,
                                                          bounds_string);}
        else{
            std::cout<<RED<<"ERROR: INITIALIZING STACK "<<i<<DEFAULT<<std::endl;
            return false;}
    }
    return true;
}

bool QPOases_sot::solve(Eigen::VectorXd &solution)
{
    for(unsigned int i = 0; i < _tasks.size(); ++i)
    {
        if(_active_stacks[i])
        {
            computeCostFunction(_tasks[i], H, g);
            if(!_qp_stack_of_tasks[i].updateTask(H, g))
                return false;

            OpenSoT::constraints::Aggregated constraints_task_i(_tasks[i]->getConstraints(), _tasks[i]->getXSize());
            if(_globalConstraints){
                constraints_task_i.getConstraintsList().push_back(_globalConstraints);
                constraints_task_i.generateAll();}
            else if(_bounds && _bounds->isConstraint())
            {
                constraints_task_i.getConstraintsList().push_back(_bounds);
                constraints_task_i.generateAll();
            }
            A = constraints_task_i.getAineq();
            lA = constraints_task_i.getbLowerBound();
            uA = constraints_task_i.getbUpperBound();
            if(i > 0)
            {
                for(unsigned int j = 0; j < i; ++j)
                {
                    if(_active_stacks[j])
                        computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A, tmp_lA, tmp_uA);
                    else
                    {
                        //Here we consider fake optimality constraints:
                        //
                        //    -1 <= 0x <= 1
                        tmp_A.setZero(_tasks[j]->getA().rows(), _tasks[j]->getA().cols());
                        tmp_lA.setConstant(_tasks[j]->getA().rows(), -1.0);
                        tmp_uA.setConstant(_tasks[j]->getA().rows(), 1.0);
                    }
                    pile(A, tmp_A);
                    pile(lA, tmp_lA);
                    pile(uA, tmp_uA);
                }
            }

            if(!_qp_stack_of_tasks[i].updateConstraints(A, lA, uA))
                return false;

            if(_bounds && _bounds->isBound()) // if it is a constraint, it has already been added in #127
            {
                constraints_task_i.getConstraintsList().push_back(_bounds);
                constraints_task_i.generateAll();
            }

            if(constraints_task_i.hasBounds()) // bounds specified everywhere will work
            {
                if(!_qp_stack_of_tasks[i].updateBounds(constraints_task_i.getLowerBound(), constraints_task_i.getUpperBound()))
                    return false;
            }

            if(!_qp_stack_of_tasks[i].solve())
                return false;

            solution = _qp_stack_of_tasks[i].getSolution();
        }
        else
        {
            //Here we do nothing
        }
    }
    return true;
}

bool QPOases_sot::setOptions(const unsigned int i, const qpOASES::Options &opt)
{
    if(i > _qp_stack_of_tasks.size()){
        std::cout<<RED<<"ERROR Index out of range!"<<DEFAULT<<std::endl;
        return false;}

    _qp_stack_of_tasks[i].setOptions(opt);
    return true;
}

bool QPOases_sot::getOptions(const unsigned int i, qpOASES::Options& opt)
{

    if(i > _qp_stack_of_tasks.size()){
        std::cout<<RED<<"Index out of range!"<<DEFAULT<<std::endl;
        return false;}

    opt = _qp_stack_of_tasks[i].getOptions();
    return true;
}

void QPOases_sot::setActiveStack(const unsigned int i, const bool flag)
{
    if(i >= 0 && i < _active_stacks.size())
        _active_stacks[i] = flag;
}

void QPOases_sot::activateAllStacks()
{
    _active_stacks.assign(_active_stacks.size(), true);
}


void QPOases_sot::_log(XBot::MatLogger::Ptr logger)
{
    for(unsigned int i = 0; i < _qp_stack_of_tasks.size(); ++i)
        _qp_stack_of_tasks[i].log(logger,i);
}
