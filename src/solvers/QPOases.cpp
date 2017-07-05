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
    if(!prepareSoT())
        throw "Can Not initizalize SoT!";
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         ConstraintPtr bounds,
                         const double eps_regularisation):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
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
    if(!prepareSoT())
        throw "Can Not initizalize SoT with bounds!";
}

void QPOases_sot::computeCostFunction(const TaskPtr& task, Eigen::MatrixXd& H, Eigen::VectorXd& g)
{
//    H = task->getA().transpose() * task->getWeight() * task->getA();
//    g = -1.0 * task->getA().transpose() * task->getWeight() * task->getb();

    if(task->getWeight().isIdentity())
    {
        H = task->getA().transpose() * task->getA();
        g = -1.0 * task->getA().transpose() * task->getb();
    }
    else
    {
        H = task->getA().transpose() * task->getWeight() * task->getA();
        g = -1.0 * task->getA().transpose() * task->getWeight() * task->getb();
    }
}

void QPOases_sot::computeOptimalityConstraint(const TaskPtr& task, OpenSoT::solvers::QPOasesProblem &problem,
                                                     Eigen::MatrixXd& A, Eigen::VectorXd& lA, Eigen::VectorXd& uA)
{
    OpenSoT::constraints::BilateralConstraint optimality_bilateral_constraint(
                task->getA(),
                task->getA()*problem.getSolution(),
                task->getA()*problem.getSolution());
    A = optimality_bilateral_constraint.getAineq();
    lA = optimality_bilateral_constraint.getbLowerBound();
    uA = optimality_bilateral_constraint.getbUpperBound();
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
                computeOptimalityConstraint(_tasks[j], _qp_stack_of_tasks[j], tmp_A, tmp_lA, tmp_uA);
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
