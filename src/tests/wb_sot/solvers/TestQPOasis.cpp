#include <gtest/gtest.h>
#include <wb_sot/solvers/QPOasis.h>
#include <yarp/sig/all.h>

namespace {

class testQPOasesProblem: public ::testing::Test,
        public wb_sot::solvers::QPOasesProblem
{
protected:

    testQPOasesProblem()
    {

    }

    void setTestProblem(const qpOASES::QProblem &problem)
    {
        _problem = problem;
    }

    virtual ~testQPOasesProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};


TEST_F(testQPOasesProblem, testSimpleProblem)
{
    yarp::sig::Matrix H(2,2); H = H.eye();
    yarp::sig::Vector g(2); g[0] = -5.0; g[1] = 5.0;
    yarp::sig::Vector x(2);
    yarp::sig::Matrix A(2,2); A.zero();
    yarp::sig::Vector l(2), u(2), lA(2), uA(2);
    l[0] = -10.0; l[1] = -10.0;
    u[0] = 10.0; u[1] = 10.0;
    lA[0] = -10.0; lA[1] = -10.0;
    uA[0] = 10.0; uA[1] = 10.0;
    qpOASES::HessianType ht = qpOASES::HST_POSDEF;
    int nWSR = 32;

    qpOASES::QProblem testProblem(x.size(), A.rows(), ht);
    this->setTestProblem(testProblem);

    this->initProblem(H, g, A, lA, uA, l, u);
    this->setNWSR(nWSR);

    EXPECT_TRUE(this->solve());
    EXPECT_TRUE(this->isQProblemInitialized());
    EXPECT_EQ(ht, this->getHessianType());
    EXPECT_EQ(nWSR, this->getNWSR());
    yarp::sig::Vector s = this->getSolution();
    EXPECT_EQ(-g[0], s[0]);
    EXPECT_EQ(-g[1], s[1]);


//    qpOASES::Options opt;
//    opt.printLevel = qpOASES::PL_HIGH;
//    opt.setToReliable();
//    opt.enableRegularisation = qpOASES::BT_TRUE;
//    opt.epsRegularisation *= 2E2;
//    testProblem.setOptions(opt);
//    testProblem.init(H.data(), g.data(), A.data(), lA.data(), uA.data(), l.data(), u.data(), nWSR, 0);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
