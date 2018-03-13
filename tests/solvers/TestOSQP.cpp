#include <OpenSoT/solvers/OSQPBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>

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

    EXPECT_TRUE(testProblemQPOASES.solve());
    EXPECT_TRUE(testProblemOSQP.solve());
    s_qpoases = testProblemQPOASES.getSolution();
    s_osqp = testProblemOSQP.getSolution();
    EXPECT_EQ(-sp.g[0], s_osqp[0]);
    EXPECT_EQ(-sp.g[1], s_osqp[1]);
    EXPECT_NEAR((s_qpoases - s_osqp).norm(),0.0, 1e-12);
    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution OSQP: "<<s_osqp.transpose()<<std::endl;

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblemOSQP.solve());
        EXPECT_TRUE(testProblemQPOASES.solve());

        Eigen::VectorXd s_osqp = testProblemOSQP.getSolution();
        Eigen::VectorXd s_qpoases = testProblemQPOASES.getSolution();
        EXPECT_EQ(-sp.g[0], s_osqp[0]);
        EXPECT_EQ(-sp.g[1], s_osqp[1]);
        EXPECT_NEAR((s_qpoases - s_osqp).norm(),0.0, 1e-12);
    }

}

void initializeIfNeeded()
{
    static bool is_initialized = false;

    if(!is_initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        srand((unsigned int)(seed));

        is_initialized = true;
    }

}

double getRandomAngle()
{
    initializeIfNeeded();
    return drand48()*2.0*M_PI-M_PI;
}

double getRandomAngle(const double min, const double max)
{
    initializeIfNeeded();
    assert(min <= max);
    if(min < -M_PI || max > M_PI)
        return getRandomAngle();

    return (double)rand()/RAND_MAX * (max-min) + min;
}

Eigen::VectorXd getRandomAngles(const Eigen::VectorXd &min,
                                               const Eigen::VectorXd &max,
                                               const int size)
{
    initializeIfNeeded();
    Eigen::VectorXd q(size);
    assert(min.size() >= size);
    assert(max.size() >= size);
    for(unsigned int i = 0; i < size; ++i)
        q(i) = getRandomAngle(min[i],max[i]);
    return q;
}

TEST_F(testOSQPProblem, testTask)
{
    Eigen::VectorXd q_ref(10); q_ref.setZero(q_ref.size());
    Eigen::VectorXd q(q_ref.size()); q.setZero(q_ref.size());
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    OpenSoT::tasks::velocity::Postural postural_task(q);
    postural_task.setReference(q_ref);
    postural_task.update(q);

    Eigen::MatrixXd H(q.size(),q.size()); H.setIdentity(H.rows(), H.cols());
    Eigen::VectorXd g(-1.0*postural_task.getb());

    OpenSoT::solvers::OSQPBackEnd qp_postural_problem(postural_task.getXSize(), 0);
    qp_postural_problem.initProblem(H,g,Eigen::MatrixXd(0,0),Eigen::VectorXd(0),Eigen::VectorXd(0),Eigen::VectorXd(0),Eigen::VectorXd(0));

    Eigen::VectorXd dq = qp_postural_problem.getSolution();

    for(unsigned int i = 0; i < 10; ++i)
    {
        q += dq;
        postural_task.update(q);

        qp_postural_problem.updateTask(H, -1.0*postural_task.getb());

        EXPECT_TRUE(qp_postural_problem.solve());
        dq = qp_postural_problem.getSolution();

        std::cout<<"dq: "<<dq.transpose()<<std::endl;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i] + dq[i], q_ref[i], 1E-12);


    std::cout<<"q+dq: "<<(q + dq).transpose()<<std::endl;
    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

