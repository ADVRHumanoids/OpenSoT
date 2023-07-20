#include <gtest/gtest.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <qpOASES/Options.hpp>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/GenericLPTask.h>

namespace {

class testGenericTask: public ::testing::Test
{
protected:

    testGenericTask()
    {
        A.resize(1,2);
        A.setRandom(1,2);
        b.resize(1);
        b.setRandom(1);
        _generic_task = std::make_shared<OpenSoT::tasks::GenericTask>(
                    OpenSoT::tasks::GenericTask("test_task",A,b));
    }

    virtual ~testGenericTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

public:

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    OpenSoT::tasks::GenericTask::Ptr _generic_task;
};

class testGenericLPTask: public ::testing::Test
{
protected:

    testGenericLPTask()
    {
        c.resize(10);
        c.setRandom(10);
        _generic_lp_task = std::make_shared<OpenSoT::tasks::GenericLPTask>(
                    OpenSoT::tasks::GenericLPTask("test_lp_task",c));
    }

    virtual ~testGenericLPTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

public:

    Eigen::VectorXd c;
    OpenSoT::tasks::GenericLPTask::Ptr _generic_lp_task;
};

class testGenericLPTaskFoo: public ::testing::Test
{
protected:

    testGenericLPTaskFoo()
    {

    }

    virtual ~testGenericLPTaskFoo() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

public:

};

TEST_F(testGenericLPTask, testMethods)
{
    Eigen::VectorXd new_c(this->_generic_lp_task->getc().size());
    new_c.setOnes(new_c.size());

    std::cout<<"new_c: "<<new_c<<std::endl;

    this->_generic_lp_task->setc(new_c);
    this->_generic_lp_task->update(Eigen::VectorXd(1));

    EXPECT_TRUE(this->_generic_lp_task->getA() == Eigen::MatrixXd::Zero(0, new_c.size()));
    EXPECT_TRUE(this->_generic_lp_task->getb() == Eigen::VectorXd::Zero(0));
    EXPECT_TRUE(this->_generic_lp_task->getc() == new_c);
    EXPECT_TRUE(this->_generic_lp_task->checkConsistency());

    std::cout<<"this->_generic_lp_task->getA(): "<<this->_generic_lp_task->getA()<<std::endl;
    std::cout<<"this->_generic_lp_task->getb(): "<<this->_generic_lp_task->getb()<<std::endl;
    std::cout<<"this->_generic_lp_task->getc(): "<<this->_generic_lp_task->getc()<<std::endl;
}

TEST_F(testGenericTask, testMethods)
{
    EXPECT_EQ(this->A, this->_generic_task->getA());
    std::cout<<"this->A: "<<this->A<<std::endl;
    std::cout<<"this->_generic_task->getA(): "<<this->_generic_task->getA()<<std::endl;
    EXPECT_EQ(this->b, this->_generic_task->getb());
    std::cout<<"this->b: "<<this->b<<std::endl;
    std::cout<<"this->_generic_task->getb(): "<<this->_generic_task->getb()<<std::endl;


    Eigen::MatrixXd newA(this->A.rows(), this->A.cols());
    newA = 2.*this->A;
    EXPECT_TRUE(this->_generic_task->setA(newA));
    this->_generic_task->update(Eigen::VectorXd(1));
    EXPECT_EQ(newA, this->_generic_task->getA());
    std::cout<<"newA: "<<newA<<std::endl;
    std::cout<<"this->_generic_task->getA(): "<<this->_generic_task->getA()<<std::endl;
    EXPECT_EQ(this->b, this->_generic_task->getb());
    std::cout<<"this->b: "<<this->b<<std::endl;
    std::cout<<"this->_generic_task->getb(): "<<this->_generic_task->getb()<<std::endl;

    Eigen::VectorXd newb(this->b.size());
    newb = 3.*this->b;
    EXPECT_TRUE(this->_generic_task->setb(newb));
    this->_generic_task->update(Eigen::VectorXd(1));
    EXPECT_EQ(newA, this->_generic_task->getA());
    std::cout<<"newA: "<<newA<<std::endl;
    std::cout<<"this->_generic_task->getA(): "<<this->_generic_task->getA()<<std::endl;
    EXPECT_EQ(newb, this->_generic_task->getb());
    std::cout<<"newb: "<<newb<<std::endl;
    std::cout<<"this->_generic_task->getb(): "<<this->_generic_task->getb()<<std::endl;

    Eigen::MatrixXd F(4,4);
    F.setRandom(4,4);
    Eigen::VectorXd g(4);
    g.setRandom(4);
    EXPECT_FALSE(this->_generic_task->setAb(F,g));
    F.setRandom(4,this->A.cols());

    EXPECT_TRUE(this->_generic_task->setAb(F,g));
    this->_generic_task->update(Eigen::VectorXd(1));
    EXPECT_EQ(F, this->_generic_task->getA());
    std::cout<<"F: "<<F<<std::endl;
    std::cout<<"this->_generic_task->getA(): "<<this->_generic_task->getA()<<std::endl;
    EXPECT_EQ(g, this->_generic_task->getb());
    std::cout<<"g: "<<g<<std::endl;
    std::cout<<"this->_generic_task->getb(): "<<this->_generic_task->getb()<<std::endl;

    Eigen::MatrixXd W = this->_generic_task->getWeight();
    EXPECT_TRUE(W.rows() == this->_generic_task->getA().rows());
    EXPECT_TRUE(W.cols() == this->_generic_task->getA().rows());
}

TEST_F(testGenericTask, testGenericTaskWithQPOASES)
{
    Eigen::MatrixXd A(1,2);
    A<<1.,1.;
    Eigen::VectorXd b(1);
    b<<1.;
    OpenSoT::tasks::GenericTask::Ptr generic_task = std::make_shared<OpenSoT::tasks::GenericTask>(
                OpenSoT::tasks::GenericTask("test_task",A,b));

    OpenSoT::AffineHelper var(2,2);
    Eigen::VectorXd u(2);
    u<<5.,5.;
    OpenSoT::constraints::GenericConstraint::Ptr bounds = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                OpenSoT::constraints::GenericConstraint("bounds",var,u,-u,OpenSoT::constraints::GenericConstraint::Type::BOUND));

    OpenSoT::AutoStack::Ptr autostack = std::make_shared<OpenSoT::AutoStack>(
                OpenSoT::AutoStack(generic_task));
    autostack<<bounds;

    OpenSoT::solvers::iHQP::Ptr sot(
        new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(),1.));


    Eigen::VectorXd x(2);
    sot->solve(x);

    std::cout<<"x: "<<x.transpose()<<std::endl;
    EXPECT_NEAR(x.sum(), b[0], 1e-9);
}

TEST_F(testGenericTask, testGenericTaskWithOSQP)
{
    Eigen::MatrixXd A(1,2);
    A<<1.,1.;
    Eigen::VectorXd b(1);
    b<<1.;
    OpenSoT::tasks::GenericTask::Ptr generic_task = std::make_shared<OpenSoT::tasks::GenericTask>(
                OpenSoT::tasks::GenericTask("test_task",A,b));

    OpenSoT::AffineHelper var(2,2);
    Eigen::VectorXd u(2);
    u<<5.,5.;
    OpenSoT::constraints::GenericConstraint::Ptr bounds = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                OpenSoT::constraints::GenericConstraint("bounds",var,u,-u,OpenSoT::constraints::GenericConstraint::Type::BOUND));

    OpenSoT::AutoStack::Ptr autostack = std::make_shared<OpenSoT::AutoStack>(
                OpenSoT::AutoStack(generic_task));
    autostack<<bounds;

    OpenSoT::solvers::iHQP::Ptr sot(
        new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(),1e-9,OpenSoT::solvers::solver_back_ends::OSQP));

    Eigen::VectorXd x(2);
    sot->solve(x);

    std::cout<<"x: "<<x.transpose()<<std::endl;
    EXPECT_NEAR(x.sum(), b[0], 1e-9);


}

TEST_F(testGenericTask, testGenericTaskVSMinimizeVariables)
{
    Eigen::MatrixXd M(12,48);
    M.setZero(12,48);
    M.leftCols(12).setIdentity();

    Eigen::VectorXd q(12);
    q.setZero(12);

    OpenSoT::AffineHelper var(M,q);

    OpenSoT::tasks::MinimizeVariable::Ptr min_var;
    min_var.reset(new OpenSoT::tasks::MinimizeVariable("min_var", var));

    OpenSoT::tasks::GenericTask::Ptr generic_min_var;
    generic_min_var.reset(new OpenSoT::tasks::GenericTask("generic_min_tas", var.getM(), var.getq()));

    min_var->update(Eigen::VectorXd(1));
    generic_min_var->update(Eigen::VectorXd(1));

    for(unsigned int i = 0; i < M.rows(); ++i)
    {
        for(unsigned int j = 0; j < M.cols(); ++j)
            EXPECT_NEAR(min_var->getA()(i,j), generic_min_var->getA()(i,j), 1e-12);
    }
    std::cout<<"min_var->getA():\n"<<min_var->getA()<<std::endl;
    std::cout<<"generic_min_var->getA():\n"<<generic_min_var->getA()<<std::endl;

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(min_var->getb()(i), generic_min_var->getb()(i), 1e-12);
    std::cout<<"min_var->getb():\n"<<min_var->getb()<<std::endl;
    std::cout<<"generic_min_var->getb():\n"<<generic_min_var->getb()<<std::endl;
}

TEST_F(testGenericLPTaskFoo, testSingleLPProblem)
{
    Eigen::MatrixXd A(2,4);
    A.setZero(2,4);
    A<<0, 1, 0, 0,
       0, 0, 0, 1;

    OpenSoT::AffineHelper var(A, Eigen::VectorXd::Zero(2));

    Eigen::VectorXd c(2);
    c.setOnes(2);

    OpenSoT::tasks::GenericLPTask::Ptr  lp_task(new OpenSoT::tasks::GenericLPTask("lp_task", c, var));

    Eigen::VectorXd expected_c(4);
    expected_c<<0,1,0,1;

     std::cout<<"lp_task A: \n"<<lp_task->getA()<<std::endl;
     std::cout<<"lp_task b: \n"<<lp_task->getb()<<std::endl;
     std::cout<<"lp_task c: \n"<<lp_task->getc()<<std::endl;

     EXPECT_TRUE(lp_task->getc() == expected_c);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
