#include <gtest/gtest.h>
#include <OpenSoT/solvers/CBCBackEnd.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/solvers/iHQP.h>
#include <chrono>

namespace {

class testCBCProblem: public ::testing::Test
{
protected:

    testCBCProblem()
    {

    }

    virtual ~testCBCProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

using namespace OpenSoT::tasks;
using namespace OpenSoT::constraints;

TEST_F(testCBCProblem, testMILPProblem)
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

    OpenSoT::solvers::CBCBackEnd::Ptr solver = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::CBC, 3, 2, task->getHessianAtype(), 0.0);
    auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(solver->initProblem(Eigen::MatrixXd(0,0), task->getc(),
                       constr->getAineq(), constr->getbLowerBound(), constr->getbUpperBound(),
                       bounds->getLowerBound(), bounds->getUpperBound()));
    auto stop = std::chrono::steady_clock::now();
    auto time_for_initProblem = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for time_for_initProblem: "<<time_for_initProblem/1000.<<" [ms]"<<std::endl;
    std::cout<<"Solution from initProblem: "<<solver->getSolution()<<std::endl;

    std::vector<double> times;
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: "<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }


    OpenSoT::solvers::CBCBackEnd::CBCBackEndOptions opt;
    opt.integer_ind.push_back(2);

    solver->setOptions(opt);
    solver->printProblemInformation(0, "", "", "");

    EXPECT_TRUE(solver->solve());
    std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

    EXPECT_DOUBLE_EQ(solver->getSolution()[0], 0.0);
    EXPECT_DOUBLE_EQ(solver->getSolution()[1], 6.0);
    EXPECT_DOUBLE_EQ(solver->getSolution()[2], 0.0);

    //WE CHANGE BOUNDS
    ub[1] = 5.5;
    solver->updateBounds(lb, ub);
    start = std::chrono::steady_clock::now();
    EXPECT_TRUE(solver->solve());
    stop = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    times.push_back(duration);

    std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;
    EXPECT_DOUBLE_EQ(solver->getSolution()[0], 0.0);
    EXPECT_DOUBLE_EQ(solver->getSolution()[1], 5.5);
    EXPECT_DOUBLE_EQ(solver->getSolution()[2], 1.0);

    opt.integer_ind.push_back(2);
    opt.integer_ind.push_back(1);

    solver->setOptions(opt);
    solver->printProblemInformation(0, "", "", "");
    start = std::chrono::steady_clock::now();
    EXPECT_TRUE(solver->solve());
    stop = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    times.push_back(duration);
    std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

    EXPECT_DOUBLE_EQ(solver->getSolution()[0], 0.25);
    EXPECT_DOUBLE_EQ(solver->getSolution()[1], 5.0);
    EXPECT_DOUBLE_EQ(solver->getSolution()[2], 1.0);

    solver->printProblemInformation(0, "", "", "");

    std::cout<<"get objective: "<<solver->getObjective()<<std::endl;

    double total_time = 0.;
    for(unsigned int i = 0; i < times.size(); ++i)
        total_time += times[i];
    std::cout<<"Average time for solve: "<<(total_time/times.size())/1000.<<" [ms]"<<std::endl;
}

TEST_F(testCBCProblem, testSetIntegerVariables)
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
    ub<<1e30, 5.5, 1.;
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


    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack;
    stack.push_back(task);


    OpenSoT::solvers::iHQP::Ptr solver;
    solver.reset(new OpenSoT::solvers::iHQP(stack, bounds, constr, 0.0,
                    OpenSoT::solvers::solver_back_ends::CBC));

    OpenSoT::solvers::BackEnd::Ptr CBCBE;
    solver->getBackEnd(0, CBCBE);

    OpenSoT::solvers::CBCBackEnd::CBCBackEndOptions opt;
    opt.integer_ind.push_back(1);
    opt.integer_ind.push_back(0);
    CBCBE->setOptions(opt);


    Eigen::VectorXd solution(3); solution.setZero(3);
    for(unsigned int i = 0; i<10; ++i)
    {
        EXPECT_TRUE(solver->solve(solution));
        std::cout<<"Solution from iHQP: "<<solution.transpose()<<std::endl;

        std::cout<<"Solution from CBCE: "<<CBCBE->getSolution().transpose()<<std::endl;

        EXPECT_TRUE(solution == CBCBE->getSolution());

        EXPECT_NEAR(solution[0], 1., 1e-9);
        EXPECT_NEAR(solution[1], 4., 1e-9);
        EXPECT_NEAR(solution[2], 0.0, 1e-9);

        CBCBE->printProblemInformation(-1,"","","");
    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
