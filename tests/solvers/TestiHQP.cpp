#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/utils/AutoStack.h>


namespace name {

class testiHQP: public OpenSoT::solvers::iHQP
{
public:
    typedef std::shared_ptr<testiHQP> Ptr;

    testiHQP(OpenSoT::AutoStack& stack_of_tasks,
             const double eps_regularisation = 0.,
             const OpenSoT::solvers::solver_back_ends be_solver = OpenSoT::solvers::solver_back_ends::qpOASES):
        OpenSoT::solvers::iHQP(stack_of_tasks, eps_regularisation, be_solver)
    {

    }

    void getCostFunction(const int level, Eigen::MatrixXd& H, Eigen::VectorXd& g)
    {
        this->computeCostFunction(this->_tasks[level], H, g);
    }

    void getCostFunctionRegularized(const int level, Eigen::MatrixXd& H, Eigen::VectorXd& g)
    {
        Eigen::MatrixXd Hr;
        Eigen::VectorXd gr;
        this->computeCostFunction(this->_tasks[level], H, g);
        computeCostFunction(this->_regularisation_task, Hr, gr);

        H += Hr;
        g += gr;
    }

private:

};

class testClass: public ::testing::Test
{
public:
    testClass()
    {

    }

    virtual ~testClass() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    OpenSoT::tasks::GenericTask::Ptr _postural;
    OpenSoT::tasks::GenericTask::Ptr _minvel;

    testiHQP::Ptr _solver;

private:


};

TEST_F(testClass, testUserRegularisation)
{
    Eigen::MatrixXd I(7,7);
    I.setIdentity();

    Eigen::VectorXd q(7), qd(7);
    q.setZero();
    qd.setOnes();

    _postural = std::make_shared<OpenSoT::tasks::GenericTask>(
                "_postural", I, (qd-q));

    _postural->update();

    OpenSoT::AutoStack::Ptr stack;
    stack /= _postural;

    _solver = std::make_shared<testiHQP>(*stack);

    Eigen::VectorXd ddq;
    EXPECT_TRUE(_solver->solve(ddq));

    std::cout<<"(qd-q): "<<(qd-q).transpose()<<std::endl;
    std::cout<<"ddq: "<<ddq.transpose()<<std::endl;

    for(unsigned int i = 0; i < _postural->getb().size(); ++i)
        EXPECT_NEAR(_postural->getb()[i], ddq[i], 1e-12);
//    EXPECT_TRUE(_postural->getb() == ddq);

    Eigen::MatrixXd H(7,7);
    Eigen::VectorXd g(7);

    _solver->getCostFunction(0,H,g);

    EXPECT_TRUE(H == _postural->getA().transpose()*_postural->getA());
    EXPECT_TRUE(g == -_postural->getA().transpose()*_postural->getb());


    Eigen::VectorXd qdot(7);
    qdot = 1e-4*qdot.setRandom();
    std::cout<<"qdot: "<<qdot.transpose()<<std::endl;
    double dt = 0.001;

    _minvel = std::make_shared<OpenSoT::tasks::GenericTask>(
                "minvel", I, -(1./dt)*qdot);
    _minvel->update();

    stack->setRegularisationTask(_minvel);

    _solver = std::make_shared<testiHQP>(*stack);

    EXPECT_TRUE(_solver->solve(ddq));

    std::cout<<"ddq: "<<ddq.transpose()<<std::endl;

    _solver->getCostFunctionRegularized(0,H,g);

    std::cout<<"Hr: "<<H<<std::endl;
    std::cout<<"gr: "<<g.transpose()<<std::endl;
    EXPECT_TRUE(H == 2*_postural->getA().transpose()*_postural->getA());
    EXPECT_TRUE(g == -(_postural->getA().transpose()*_postural->getb() + _minvel->getA().transpose()*_minvel->getb()));


    Eigen::VectorXd sol = -H.inverse()*g;
    std::cout<<"sol: "<<sol.transpose()<<std::endl;

    for(unsigned int i = 0; i < ddq.size(); ++i)
        EXPECT_NEAR(ddq[i], sol[i], 1e-12);

//    EXPECT_TRUE(ddq == sol);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
