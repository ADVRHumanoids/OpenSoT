#include <gtest/gtest.h>
#include <wb_sot/solvers/QPOases.h>
#include <yarp/sig/all.h>

namespace {

class simpleProblem
{
public:
    simpleProblem():
        H(2,2),
        g(2),
        A(2,2),
        l(2), u(2), lA(2), uA(2),
        ht(qpOASES::HST_IDENTITY)
    {
        H = H.eye();
        g[0] = -5.0; g[1] = 5.0;
        A.zero();
        l[0] = -10.0; l[1] = -10.0;
        u[0] = 10.0; u[1] = 10.0;
        lA[0] = -10.0; lA[1] = -10.0;
        uA[0] = 10.0; uA[1] = 10.0;

    }

    yarp::sig::Matrix H;
    yarp::sig::Vector g;
    yarp::sig::Matrix A;
    yarp::sig::Vector l;
    yarp::sig::Vector u;
    yarp::sig::Vector lA;
    yarp::sig::Vector uA;
    qpOASES::HessianType ht;
};

class testQPOasesProblem: public ::testing::Test,
        public wb_sot::solvers::QPOasesProblem
{
protected:

    testQPOasesProblem()
    {

    }

    void setTestProblem(const qpOASES::SQProblem &problem)
    {
        this->setProblem(problem);
    }

    virtual ~testQPOasesProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

/**
 * @brief TEST_F testSimpleProblem test solution of a simple CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testSimpleProblem)
{
    yarp::sig::Vector x(2);
    simpleProblem sp;

    qpOASES::SQProblem testProblem(x.size(), sp.A.rows(), sp.ht);
    this->setTestProblem(testProblem);

    this->initProblem(sp.H, sp.g, sp.A, sp.lA, sp.uA, sp.l, sp.u);

    EXPECT_TRUE(this->solve());
    EXPECT_TRUE(this->isQProblemInitialized());
    yarp::sig::Vector s = this->getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(this->solve());

        yarp::sig::Vector s = this->getSolution();
        EXPECT_EQ(-sp.g[0], s[0]);
        EXPECT_EQ(-sp.g[1], s[1]);
    }

}

/**
 * @brief TEST_F testUpdatedProblem test solution of a simple NON-CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testUpdatedProblem)
{
    yarp::sig::Vector x(2);
    simpleProblem sp;

    qpOASES::SQProblem testProblem(x.size(), sp.A.rows(), sp.ht);
    this->setTestProblem(testProblem);

    this->initProblem(sp.H, sp.g, sp.A, sp.lA, sp.uA, sp.l, sp.u);

    EXPECT_TRUE(this->solve());
    yarp::sig::Vector s = this->getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    sp.g[0] = -1.0; sp.g[1] = 1.0;
    EXPECT_TRUE(this->updateTask(sp.H, sp.g));
    EXPECT_TRUE(this->solve());

    s = this->getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);
}

/**
 * @brief TEST_F testAddProblem test solution of a simple NON-CONSTANT QP problem
 * with variable size of H, g, A, lA, uA, l and u.
 */
TEST_F(testQPOasesProblem, testAddProblem)
{
    yarp::sig::Vector x(3);

    yarp::sig::Matrix H(2,3); H.zero(); H(0,0) = 1.0; H(1,1) = 1.0;
    yarp::sig::Matrix H_new(1,3); H_new.zero(); H_new(0,2) = 1.0;

    yarp::sig::Vector g(2); g[0] = 5.0; g[1] = -5.0;
    yarp::sig::Vector g_new(1); g_new[0] = 2.0;

    yarp::sig::Matrix A(2,3); A.zero();
    yarp::sig::Matrix A_new(1,3); A_new.zero();

    yarp::sig::Vector l(2, -10.0);
    yarp::sig::Vector l_new(1, -10.0);

    yarp::sig::Vector u(2, 10.0);
    yarp::sig::Vector u_new(1, 10.0);

    yarp::sig::Vector lA; lA = l;
    yarp::sig::Vector lA_new; lA_new = l_new;

    yarp::sig::Vector uA; uA = u;
    yarp::sig::Vector uA_new; uA_new = u_new;
    qpOASES::HessianType ht = qpOASES::HST_IDENTITY;


    qpOASES::SQProblem testProblem(x.size(), A.rows(), ht);
    this->setTestProblem(testProblem);

    this->initProblem(H, g, A, lA, uA, l, u);

    EXPECT_TRUE(this->solve());
    yarp::sig::Vector s1 = this->getSolution();
    EXPECT_EQ(-g[0], s1[0]);
    EXPECT_EQ(-g[1], s1[1]);
    std::cout<<"s1 size: "<<s1.size()<<std::endl;
    std::cout<<"s1 solution: ["<<s1[0]<<" "<<s1[1]<<" "<<s1[2]<<"]"<<std::endl;

    this->setnWSR(127);
    EXPECT_TRUE(this->addProblem(H_new, g_new, A_new, lA_new, uA_new, l_new, u_new));
    yarp::sig::Vector s2 = this->getSolution();
    EXPECT_EQ(-g[0], s2[0]);
    EXPECT_EQ(-g[1], s2[1]);
    EXPECT_EQ(-g_new[0], s2[2]);
    std::cout<<"s2 size: "<<s2.size()<<std::endl;
    std::cout<<"s2 solution: ["<<s2[0]<<" "<<s2[1]<<" "<<s2[2]<<"]"<<std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
