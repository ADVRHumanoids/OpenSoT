#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/BilateralConstraint.h>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace yarp::math;
using namespace OpenSoT::solvers;

QPOases_sot::QPOases_sot(Stack &stack_of_tasks, const double eps_regularisation):
    Solver(stack_of_tasks),
    _epsRegularisation(eps_regularisation)
{
    if(!prepareSoT())
        throw "Can Not initizalize SoT!";
}

QPOases_sot::QPOases_sot(Stack &stack_of_tasks,
                         ConstraintPtr bounds, const double eps_regularisation):
    Solver(stack_of_tasks, bounds),
    _epsRegularisation(eps_regularisation)
{
    if(!prepareSoT())
        throw "Can Not initizalize SoT with bounds!";
}

void QPOases_sot::computeVelCtrlCostFunction(const TaskPtr& task, yarp::sig::Matrix& H, yarp::sig::Vector& g)
{
    H = task->getA().transposed() * task->getWeight() * task->getA();
    g = -1.0 * task->getLambda() * task->getA().transposed() * task->getWeight() * task->getb();
}

void QPOases_sot::computeVelCtrlOptimalityConstraint(const TaskPtr& task, OpenSoT::solvers::QPOasesProblem &problem,
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

bool QPOases_sot::prepareSoT()
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

        QPOasesProblem problem_i(_tasks[i]->getXSize(), A.rows(), (OpenSoT::HessianType)(_tasks[i]->getHessianAtype()),
                                 _epsRegularisation);

        if(problem_i.initProblem(H, g, A, lA, uA, l, u)){
            _qp_stack_of_tasks.push_back(problem_i);
            _qp_stack_of_tasks[i].printProblemInformation(i, _tasks[i]->getTaskID());}
        else{
            std::cout<<RED<<"ERROR: INITIALIZING STAK "<<i<<DEFAULT<<std::endl;
            return false;}
    }
    return true;
}

bool QPOases_sot::solve(Vector &solution)
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
