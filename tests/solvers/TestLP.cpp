#include <gtest/gtest.h>
#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/GenericLPTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/solvers/CBCBackEnd.h>

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
    std::cout<<"lp_task->getc(): "<<lp_task->getc()<<std::endl;

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

//// This test is interisting since minimize a Quadratic task using the L1 norm:
///
///     min ||Ax-b||_2 + ||x||_1
///
TEST_F(testLProblem, testQuadraticLPProblem)
{
    OpenSoT::OptvarHelper::VariableVector vars;
    vars.emplace_back("x", 4);
    vars.emplace_back("t", 4);

    OpenSoT::OptvarHelper opt(vars);

    Eigen::MatrixXd A(1, 4);
    A<<1,2,3,4;
    Eigen::VectorXd b(1);
    b<<10;
    OpenSoT::tasks::GenericTask::Ptr quadratic_task(
        new OpenSoT::tasks::GenericTask("quadratic_task", A, b, opt.getVariable("x")));

    std::cout<<"quadratic_task->getA(): "<<quadratic_task->getA()<<std::endl;
    std::cout<<"quadratic_task->getb(): "<<quadratic_task->getb()<<std::endl;


    Eigen::MatrixXd _A(4,4);
    _A.setZero(4,4);
    Eigen::VectorXd _b(4);
    _b.setZero(4);
    OpenSoT::tasks::GenericTask::Ptr lp_task(
        new OpenSoT::tasks::GenericTask("lp_task", _A, _b, opt.getVariable("t")));
    Eigen::VectorXd c(4);
    c.setOnes(4);
    c *= 10.;
    lp_task->setc(c);
    lp_task->setHessianType(OpenSoT::HST_ZERO);
    std::cout<<"lp_task->getA(): "<<lp_task->getA()<<std::endl;
    std::cout<<"lp_task->getb(): "<<lp_task->getb()<<std::endl;
    std::cout<<"lp_task->getc(): "<<lp_task->getc()<<std::endl;

    OpenSoT::AffineHelper Aineq1 = opt.getVariable("x")-opt.getVariable("t");
    Eigen::VectorXd lb(4);
    lb<<-1e30,-1e30,-1e30,-1e30;
    Eigen::VectorXd ub(4);
    ub.setZero(4);
    OpenSoT::constraints::GenericConstraint::Ptr lasso_constr_1(
                new OpenSoT::constraints::GenericConstraint("lasso_constr_1", Aineq1, ub, lb,
                                                            OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    OpenSoT::AffineHelper Aineq2 = opt.getVariable("x")+opt.getVariable("t");
    Eigen::VectorXd lb2(4);
    lb2.setZero(4);
    Eigen::VectorXd ub2(4);
    ub2<<1e30,1e30,1e30,1e30;
    OpenSoT::constraints::GenericConstraint::Ptr lasso_constr_2(
                new OpenSoT::constraints::GenericConstraint("lasso_constr_2", Aineq2, ub2, lb2,
                                                            OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    Eigen::VectorXd zeros(4);
    zeros.setZero(4);
    Eigen::VectorXd inf(4);
    inf.setOnes(4);
    inf*=1e30;
    OpenSoT::constraints::GenericConstraint::Ptr lasso_positive(
                new OpenSoT::constraints::GenericConstraint("lasso_positive", opt.getVariable("t"), inf, zeros,
                                                            OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));

    OpenSoT::AutoStack::Ptr autostack(new OpenSoT::AutoStack(quadratic_task+lp_task));
    autostack = autostack<<lasso_constr_1<<lasso_constr_2<<lasso_positive;

    std::cout<<"autostack->getStack()[0]->getA(): \n"<<autostack->getStack()[0]->getA()<<std::endl;

    OpenSoT::solvers::iHQP sot2(autostack->getStack(), autostack->getBounds(),0.0, OpenSoT::solvers::solver_back_ends::OSQP);
    Eigen::VectorXd solution(8);
    EXPECT_TRUE(sot2.solve(solution));
    std::cout<<"OSQP Solution: "<<solution.head(4).transpose()<<std::endl;

    OpenSoT::solvers::BackEnd::Ptr be;
    EXPECT_TRUE(sot2.getBackEnd(0, be));
    std::cout<<"H: "<<be->getH()<<std::endl;
    std::cout<<"g: "<<be->getg()<<std::endl;
    std::cout<<"A: "<<be->getA()<<std::endl;
    std::cout<<"l: "<<be->getlA()<<std::endl;
    std::cout<<"u: "<<be->getuA()<<std::endl;

    EXPECT_NEAR(solution[0], 0.0, 1e-4);
    EXPECT_NEAR(solution[1], 0.0, 1e-4);
    EXPECT_NEAR(solution[2], 0.0, 1e-4);
    EXPECT_NEAR(solution[3], 1.875, 1e-4);


    OpenSoT::solvers::iHQP sot(autostack->getStack(), autostack->getBounds(),0.0, OpenSoT::solvers::solver_back_ends::qpOASES);
    EXPECT_TRUE(sot.solve(solution));
    std::cout<<"qpOASES Solution: "<<solution.head(4).transpose()<<std::endl;

    EXPECT_NEAR(solution[0], 0.0, 1e-4);
    EXPECT_NEAR(solution[1], 0.0, 1e-4);
    EXPECT_NEAR(solution[2], 0.0, 1e-4);
    EXPECT_NEAR(solution[3], 1.875, 1e-4);

}

TEST_F(testLProblem, testMILPProblem)
{
    Eigen::MatrixXd A_qp(1,3); A_qp << 4., 2., 0.;
    Eigen::VectorXd b_qp(1); b_qp<<12;
    OpenSoT::tasks::GenericTask::Ptr task_qp(new OpenSoT::tasks::GenericTask("task_qp",A_qp,b_qp));
    task_qp->update(Eigen::VectorXd(1));

    //     Eigen::VectorXd c_qp(2);
    //     c_qp<<4.,2;
    //     GenericLPTask::Ptr task_qp(new GenericLPTask("task_qp",c_qp));
    //     task_qp->update(Eigen::VectorXd(1));

    Eigen::VectorXd c_CBC(3);
    c_CBC<<-3.,-2.,-1;
    OpenSoT::tasks::GenericLPTask::Ptr task_CBC(new OpenSoT::tasks::GenericLPTask("task_CBC",c_CBC));
    task_CBC->update(Eigen::VectorXd(1));

    Eigen::VectorXd lb(3), ub(3);
    lb.setZero(3);
    ub<<1e30, 1e30, 1.;
    OpenSoT::constraints::GenericConstraint::Ptr bounds(new OpenSoT::constraints::GenericConstraint("bounds", ub, lb, 3));
    bounds->update(Eigen::VectorXd(1));

    Eigen::MatrixXd Ac_CBC(1,3);
    Ac_CBC<<1.,1.,1;
    Eigen::VectorXd lA_CBC(1), uA_CBC(1);
    lA_CBC<<-1e30;
    uA_CBC<<7.;

    OpenSoT::AffineHelper var(Ac_CBC, Eigen::VectorXd::Zero(1));
    OpenSoT::constraints::GenericConstraint::Ptr constr(
                             new OpenSoT::constraints::GenericConstraint("constraint", var, uA_CBC, lA_CBC,
                                                            OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));
    constr->update(Eigen::VectorXd(1));

    /* _autostack */
    OpenSoT::AutoStack::Ptr _autostack;
    task_CBC  << constr;
    _autostack = (task_qp/task_CBC);
    _autostack << bounds;

    /* FrontEnd (iHQP) with multiple solvers */
    OpenSoT::solvers::solver_back_ends solver_1 = OpenSoT::solvers::solver_back_ends::qpOASES;
    OpenSoT::solvers::solver_back_ends solver_2 = OpenSoT::solvers::solver_back_ends::OSQP;
    OpenSoT::solvers::solver_back_ends solver_3 = OpenSoT::solvers::solver_back_ends::CBC;

    std::vector<OpenSoT::solvers::solver_back_ends> solver_vector(2);
    solver_vector[0]=solver_2; solver_vector[1]=solver_3;

    OpenSoT::solvers::CBCBackEnd::CBCBackEndOptions opt_CBC;
    opt_CBC.integer_ind.push_back(2);

    OpenSoT::solvers::iHQP::Ptr solver;
    solver = boost::make_shared<OpenSoT::solvers::iHQP>(_autostack->getStack(), _autostack->getBounds(), 1.0, solver_vector);

    OpenSoT::solvers::BackEnd::Ptr CBC_Ptr;
    solver->getBackEnd(1,CBC_Ptr);
    CBC_Ptr->setOptions(opt_CBC);


    CBC_Ptr->printProblemInformation(0, "", "", "");

    Eigen::VectorXd sol(3); sol.setZero(3);
    EXPECT_TRUE(solver->solve(sol));
    std::cout<<"Solution from BE solve: \n"<<CBC_Ptr->getSolution()<<std::endl;
    std::cout<<"Solution from FE solve: \n"<<sol<<std::endl;

    EXPECT_NEAR(sol[0], 0.0, 1e-9);
    EXPECT_NEAR(sol[1], 6.0, 1e-9);
    EXPECT_NEAR(sol[2], 1.0, 1e-9);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
