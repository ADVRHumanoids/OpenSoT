#include <gtest/gtest.h>
#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>

namespace {

class testLProblem: public ::testing::Test
{
protected:

    testLProblem()
    {

    }

    virtual ~testLProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testLProblem, testSingleLPProblem)
{
    Eigen::MatrixXd A(4,4);
    A.setZero(4,4);
    Eigen::VectorXd b(4);
    b.setZero(4);

    Eigen::VectorXd c(4);
    c.setOnes(4);

    OpenSoT::tasks::GenericTask::Ptr  lp_task(new OpenSoT::tasks::GenericTask("lp_task", A, b));
    lp_task->update(Eigen::VectorXd(4));
    lp_task->setHessianType(OpenSoT::HessianType::HST_ZERO);
    lp_task->setc(c);

    Eigen::VectorXd lb(4), ub(4);
    lb = -2*Eigen::VectorXd::Ones(4);
    ub = 2.*Eigen::VectorXd::Ones(4);
    OpenSoT::constraints::GenericConstraint::Ptr constr(new OpenSoT::constraints::GenericConstraint("bounds", ub, lb, 4));
    constr->update(Eigen::VectorXd(4));
    std::cout<<"lb: "<<constr->getLowerBound().transpose()<<std::endl;
    std::cout<<"ub: "<<constr->getUpperBound().transpose()<<std::endl;

    OpenSoT::AutoStack::Ptr autostack(new OpenSoT::AutoStack(lp_task));
    autostack = autostack<<constr;

    OpenSoT::solvers::iHQP::Stack tasks = autostack->getStack();
    EXPECT_EQ(tasks[0]->getHessianAtype(), OpenSoT::HessianType::HST_ZERO);

    OpenSoT::solvers::iHQP sot(autostack->getStack(), autostack->getBounds(),0.0);

    OpenSoT::solvers::BackEnd::Ptr be;
    sot.getBackEnd(0, be);
    std::cout<<"H: "<<be->getH()<<std::endl;
    std::cout<<"g: "<<be->getg().transpose()<<std::endl;
    std::cout<<"lb: "<<be->getl().transpose()<<std::endl;
    std::cout<<"ub: "<<be->getu().transpose()<<std::endl;

    Eigen::VectorXd solution(4);
    EXPECT_TRUE(sot.solve(solution));
    std::cout<<"qpOASES Solution: "<<solution.transpose()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
        EXPECT_NEAR(solution[i], lb[i], 1e-8);

    OpenSoT::solvers::iHQP sot2(autostack->getStack(), autostack->getBounds(),0.0, OpenSoT::solvers::solver_back_ends::OSQP);

    OpenSoT::solvers::BackEnd::Ptr be2;
    sot2.getBackEnd(0, be2);
    std::cout<<"H: "<<be2->getH()<<std::endl;
    std::cout<<"g: "<<be2->getg().transpose()<<std::endl;
    std::cout<<"lb: "<<be2->getl().transpose()<<std::endl;
    std::cout<<"ub: "<<be2->getu().transpose()<<std::endl;

    EXPECT_TRUE(sot2.solve(solution));
    std::cout<<"OSQP Solution: "<<solution.transpose()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
        EXPECT_NEAR(solution[i], lb[i], 1e-8);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
