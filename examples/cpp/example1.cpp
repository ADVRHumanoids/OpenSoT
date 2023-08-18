#include <memory>

#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>

#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

/**
 * @brief This example shows a basic usage of generic tasks and constraints to setupe a single static QP
 * using OpenSoT.
 */

int main()
{
    using namespace OpenSoT::solvers;

    /**
     * @brief Creates A and b matrices which will be used in the cost function:
     *
     *  T := ||Ax - b||
     */
    Eigen::MatrixXd A(5,5);
    Eigen::VectorXd b(5);

    A.setIdentity(5, 5);
    b.setConstant(5, 2.0);

    /**
     * @brief Creates 2 generic OpenSoT tasks using the same A and b functions
     */
    auto mytask1  = std::make_shared<OpenSoT::tasks::GenericTask>("mytask1", A, b);
    auto mytask2 = std::make_shared<OpenSoT::tasks::GenericTask>("mytask2", A.Random(5,5), b.Random(5));

    /**
     * @brief Creates a constraint:
     *
     *  C := lb <= Gx + c <= ub
     */
    Eigen::MatrixXd G(1, 5);
    G << 1, 1, 1, 1, 1;

    Eigen::VectorXd c(1);
    c.setZero(1);

    Eigen::VectorXd ub(1), lb(1);
    ub << 0.3;
    lb << -1.0;

    /**
     * @brief Creates a generic OpenSoT constraint
     */
    OpenSoT::AffineHelper y(G, c);
    auto constraint   = std::make_shared<OpenSoT::constraints::GenericConstraint>("mytask",y, ub, lb,
                    OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    /**
     * @brief Creates a OpenSoT stack putting toghether tasks and constraints
     */
    OpenSoT::AutoStack::Ptr stack = std::make_shared<OpenSoT::AutoStack>(mytask1 + mytask2); // The cost function is the sum of two tasks
    stack << constraint; // Constraints are added to the stack
    stack->update(Eigen::VectorXd(0)); // Stack update should be called every time tasks or constraints changes

    /**
     * @brief The stack is inserted into a solver using the iHQP technique (inequality Hierarchical QP)
     */

    double eps_regularisation = 2e2;
    solver_back_ends be_solver = solver_back_ends::qpOASES; // iHQP will use qpOASES as back-end solver
    iHQP solver(*stack, eps_regularisation, be_solver);

    /**
     * @brief solve is called to compute a solution
     */
    Eigen::VectorXd solution;
    if(!solver.solve(solution))
    {
        std::cout << "unable to solve" << std::endl;
        exit(1);
    }


    /** outputs **/
    std::cout<<"Stack levels: "<< stack->getStack().size()<<std::endl;
    std::cout<<"Cost function A: \n"<< stack->getStack()[0]->getA()<<std::endl;
    std::cout<<"Cost function b: \n"<< stack->getStack()[0]->getb()<<std::endl;
    std::cout<<"Number of Constraints: "<<stack->getBoundsList().size()<<std::endl;
    std::cout<<"Constraint Matrix: \n"<<stack->getBounds()->getAineq()<<std::endl;
    std::cout<<"Constraint ub: \n"<<stack->getBounds()->getbUpperBound()<<std::endl;
    std::cout<<"Constraint lb: \n"<<stack->getBounds()->getbLowerBound()<<std::endl;
    std::cout << "Solution is " << solution.transpose() << std::endl;

    return 0;
}
