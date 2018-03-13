#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <gtest/gtest.h>

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
        H.setIdentity(H.rows(), H.cols());
        g[0] = -5.0; g[1] = 5.0;
        A.setZero(A.rows(), A.cols());
        l[0] = -10.0; l[1] = -10.0;
        u[0] = 10.0; u[1] = 10.0;
        lA[0] = -10.0; lA[1] = -10.0;
        uA[0] = 10.0; uA[1] = 10.0;

    }

    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    Eigen::MatrixXd A;
    Eigen::VectorXd l;
    Eigen::VectorXd u;
    Eigen::VectorXd lA;
    Eigen::VectorXd uA;
    qpOASES::HessianType ht;
};

class testOSQPProblem: public ::testing::Test
{
protected:

    testOSQPProblem()
    {

    }

    virtual ~testOSQPProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testOSQPProblem, testSimpleProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::QPOasesBackEnd testProblemQPOASES(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);
    OpenSoT::solvers::OSQPBackEnd testProblemOSQP(x.size(), sp.A.rows());

    EXPECT_TRUE(testProblemQPOASES.initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    EXPECT_TRUE(testProblemOSQP.initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    Eigen::VectorXd s_qpoases = testProblemQPOASES.getSolution();
    Eigen::VectorXd s_osqp = testProblemOSQP.getSolution();

    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution OSQP: "<<s_osqp.transpose()<<std::endl;

    EXPECT_NEAR((s_qpoases - s_osqp).norm(),0.0, 1e-12);

//    EXPECT_TRUE(testProblem.solve());
//    Eigen::VectorXd s_qpoases = testProblem.getSolution();
//    EXPECT_EQ(-sp.g[0], s[0]);
//    EXPECT_EQ(-sp.g[1], s[1]);

//    for(unsigned int i = 0; i < 10; ++i)
//    {
//        EXPECT_TRUE(testProblem.solve());

//        Eigen::VectorXd s = testProblem.getSolution();
//        EXPECT_EQ(-sp.g[0], s[0]);
//        EXPECT_EQ(-sp.g[1], s[1]);
//    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

