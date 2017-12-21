#include <qpOASES.hpp>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <XBotInterface/Logger.hpp>

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
        throw std::runtime_error("Can Not initizalize SoT!");
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
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
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
        throw std::runtime_error("Can Not initizalize SoT with bounds!");
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
                                                Eigen::MatrixXd& A, Eigen::VectorXd& lA, Eigen::VectorXd& uA)
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

        A.set(constraints_task_i.getAineq());
        lA.set(constraints_task_i.getbLowerBound());
        uA.set(constraints_task_i.getbUpperBound());
        if(i > 0)
        {
            Eigen::MatrixXd _tmp_A;
            Eigen::VectorXd _tmp_lA, _tmp_uA;
            for(unsigned int j = 0; j < i; ++j)
            {
                computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], _tmp_A, _tmp_lA, _tmp_uA);

                if( j == i-1)
                {
                    tmp_A.push_back(_tmp_A);
                    tmp_lA.push_back(_tmp_lA);
                    tmp_uA.push_back(_tmp_uA);
                }

                if(!constraints_str.compare("") == 0)
                    constraints_str = constraints_str + "+";
                constraints_str = constraints_str + _tasks[j]->getTaskID() + "_optimality";

                A.pile(tmp_A[j]);
                lA.pile(tmp_lA[j]);
                uA.pile(tmp_uA[j]);
            }
        }

        if(_bounds && _bounds->isBound()){   // if it is a constraint, it has already been added in #74
            constraints_task_i.getConstraintsList().push_back(_bounds);
            constraints_task_i.generateAll();}
        l = constraints_task_i.getLowerBound();
        u = constraints_task_i.getUpperBound();

        QPOasesProblem problem_i(_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
                                 _epsRegularisation);

        if(problem_i.initProblem(H, g, A.generate_and_get(), lA.generate_and_get(), uA.generate_and_get(), l, u)){
            _qp_stack_of_tasks.push_back(problem_i);
            std::string bounds_string = "";
            if(_bounds)
                bounds_string = _bounds->getConstraintID();
            _qp_stack_of_tasks[i].printProblemInformation(i, _tasks[i]->getTaskID(),
                                                          constraints_str,
                                                          bounds_string);}
        else{
            XBot::Logger::error("ERROR: INITIALIZING STACK %i \n", i);
            return false;}

        constraints_task.push_back(constraints_task_i);
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

            OpenSoT::constraints::Aggregated& constraints_task_i = constraints_task[i];
            constraints_task_i.generateAll();

            A.set(constraints_task_i.getAineq());
            lA.set(constraints_task_i.getbLowerBound());
            uA.set(constraints_task_i.getbUpperBound());
            if(i > 0)
            {
                for(unsigned int j = 0; j < i; ++j)
                {
                    if(_active_stacks[j])
                        computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A[j], tmp_lA[j], tmp_uA[j]);
                    else
                    {
                        //Here we consider fake optimality constraints:
                        //
                        //    -1 <= 0x <= 1
                        tmp_A[j].setZero(_tasks[j]->getA().rows(), _tasks[j]->getA().cols());
                        tmp_lA[j].setConstant(_tasks[j]->getA().rows(), -1.0);
                        tmp_uA[j].setConstant(_tasks[j]->getA().rows(), 1.0);
                    }
                    A.pile(tmp_A[j]);
                    lA.pile(tmp_lA[j]);
                    uA.pile(tmp_uA[j]);
                }
            }

            if(!_qp_stack_of_tasks[i].updateConstraints(A.generate_and_get(),
                                    lA.generate_and_get(), uA.generate_and_get()))
                return false;


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
        XBot::Logger::error("ERROR Index out of range! \n");
        return false;}

    _qp_stack_of_tasks[i].setOptions(opt);
    return true;
}

bool QPOases_sot::getOptions(const unsigned int i, qpOASES::Options& opt)
{

    if(i > _qp_stack_of_tasks.size()){
        XBot::Logger::error("ERROR Index out of range! \n");
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
