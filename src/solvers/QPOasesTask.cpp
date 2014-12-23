#include <OpenSoT/solvers/QPOasesTask.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <qpOASES/Options.hpp>

using namespace yarp::math;

using namespace OpenSoT::solvers;

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

QPOasesTask::QPOasesTask(const boost::shared_ptr<Task<Matrix, Vector> > &task, const double eps_regularisation):
    QPOasesProblem(task->getXSize(),
                   OpenSoT::constraints::Aggregated(task->getConstraints(), task->getXSize()).getAineq().rows(),
                   (OpenSoT::HessianType)(task->getHessianAtype()), eps_regularisation),
    _task(task)
{
    prepareTaskData(true);
    printProblemInformation(-1);
    assert(initProblem(_H, _g, _A, _lA, _uA, _l, _u));
}

OpenSoT::solvers::QPOasesTask::QPOasesTask(const boost::shared_ptr<Task<Matrix, Vector> > &task, const boost::shared_ptr<Constraint<Matrix, Vector> > &bounds,
                                           const double eps_regularisation):
    QPOasesProblem(task->getXSize(),
                   OpenSoT::constraints::Aggregated(task->getConstraints(), task->getXSize()).getAineq().rows(),
                   (OpenSoT::HessianType)(task->getHessianAtype()), eps_regularisation),
    _task(task)
{
    prepareTaskData(true);
    _l = bounds->getLowerBound();
    _u = bounds->getUpperBound();
    printProblemInformation(-1);
    assert(initProblem(_H, _g, _A, _lA, _uA, _l, _u));
}

QPOasesTask::~QPOasesTask()
{}

void QPOasesTask::prepareTaskData(bool update_constraints_and_bounds)
{
    /* Set Hessian Type */
    setHessianType(_task->getHessianAtype());

    /* Compute cost function */
    velocityControlCostFunction();

    if(update_constraints_and_bounds)
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

void QPOasesTask::velocityControlCostFunction()
{
    _H = _task->getA().transposed() * _task->getWeight() * _task->getA();
    _g = -1.0 * _task->getLambda() * _task->getA().transposed() * _task->getWeight() * _task->getb();
}

bool QPOasesTask::solve(bool update_constraints)
{
    prepareTaskData(update_constraints);
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
