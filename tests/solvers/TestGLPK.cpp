#include <gtest/gtest.h>
#include <OpenSoT/solvers/GLPKBackEnd.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <chrono>

using namespace OpenSoT::constraints;
using namespace OpenSoT::tasks;

namespace {

class testGLPKProblem: public ::testing::Test
{
protected:

    testGLPKProblem()
    {

    }

    virtual ~testGLPKProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testGLPKProblem, testMILPProblem)
{
    Eigen::MatrixXd A(3,3); A.setZero(3,3);
    Eigen::VectorXd b(3); b.setZero(3);
    GenericTask::Ptr task(new GenericTask("task",A,b));
    Eigen::VectorXd c(3);
    c<<-3.,-2.,-1;
    task->setHessianType(OpenSoT::HST_ZERO);
    task->setc(c);
    task->update(Eigen::VectorXd(1));

    Eigen::VectorXd lb(3), ub(3);
    lb.setZero(3);
    ub<<1e30, 1e30, 1.;
    GenericConstraint::Ptr bounds(new GenericConstraint("bounds", ub, lb, 3));
    bounds->update(Eigen::VectorXd(1));

    Eigen::MatrixXd Ac(2,3);
    Ac<<1.,1.,1.,
        4.,2.,1.;
    Eigen::VectorXd lA(2), uA(2);
    lA<<-1e30, 12.;
    uA<<7., 12.;

    OpenSoT::AffineHelper var(Ac, Eigen::VectorXd::Zero(2));
    GenericConstraint::Ptr constr(new GenericConstraint("constraint", var, uA, lA, GenericConstraint::Type::CONSTRAINT));
    constr->update(Eigen::VectorXd(1));
    std::cout<<"lA: "<<constr->getbLowerBound();

    OpenSoT::solvers::GLPKBackEnd::Ptr solver = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::GLPK, 3, 2, task->getHessianAtype(), 0.0);



    auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(solver->initProblem(Eigen::MatrixXd(0,0), task->getc(),
                       constr->getAineq(), constr->getbLowerBound(), constr->getbUpperBound(),
                       bounds->getLowerBound(), bounds->getUpperBound()));
    auto stop = std::chrono::steady_clock::now();
    auto time_for_initProblem = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for time_for_initProblem: "<<time_for_initProblem/1000.<<" [ms]"<<std::endl;
    std::cout<<"Solution from initProblem: \n"<<solver->getSolution()<<std::endl;

    std::vector<double> times;
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0],0.0, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1],5.5, 1e-12);
    EXPECT_NEAR(solver->getSolution()[2],1.0, 1e-12);

    ub[1] = 5;
    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0],0.25, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1],5., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2],1.0, 1e-12);

    ub[1] = 9.1;

    OpenSoT::solvers::GLPKBackEnd::GLPKBackEndOptions opt;
    opt.var_id_kind_.push_back(std::pair<int,int>(1,GLP_IV));
    opt.var_id_kind_.push_back(std::pair<int,int>(2,GLP_BV));
    opt.ROUND_BOUNDS = -1;
    solver->setOptions(opt);
    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 0., 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], 6., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 0., 1e-12);

    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 0., 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], 6., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 0., 1e-12);

    lb[1] = -2.1;
    ub[1] = -1.;

    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 3.25, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], -1., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 1., 1e-12);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

