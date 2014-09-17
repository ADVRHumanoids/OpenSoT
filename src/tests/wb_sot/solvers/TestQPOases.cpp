#include <gtest/gtest.h>
#include <wb_sot/solvers/QPOases.h>
#include <yarp/sig/all.h>
#include <drc_shared/idynutils.h>
#include <wb_sot/tasks/velocity/Postural.h>
#include <drc_shared/tests_utils.h>
#include <yarp/math/Math.h>
#include <wb_sot/bounds/Aggregated.h>
#include <wb_sot/tasks/velocity/Postural.h>
#include <wb_sot/bounds/velocity/JointLimits.h>

using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

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

class testQPOasesTask: public ::testing::Test
{
protected:

    testQPOasesTask()
    {

    }

    virtual ~testQPOasesTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testQPOases_sot: public ::testing::Test
{
protected:

    testQPOases_sot()
    {

    }

    virtual ~testQPOases_sot() {

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
    std::cout<<GREEN<<"s1 size: "<<s1.size()<<DEFAULT<<std::endl;
    std::cout<<GREEN<<"s1 solution: ["<<s1[0]<<" "<<s1[1]<<" "<<s1[2]<<"]"<<DEFAULT<<std::endl;

    this->setnWSR(127);
    EXPECT_TRUE(this->addProblem(H_new, g_new, A_new, lA_new, uA_new, l_new, u_new));
    yarp::sig::Vector s2 = this->getSolution();
    EXPECT_EQ(-g[0], s2[0]);
    EXPECT_EQ(-g[1], s2[1]);
    EXPECT_EQ(-g_new[0], s2[2]);
    std::cout<<GREEN<<"s2 size: "<<s2.size()<<DEFAULT<<std::endl;
    std::cout<<GREEN<<"s2 solution: ["<<s2[0]<<" "<<s2[1]<<" "<<s2[2]<<"]"<<DEFAULT<<std::endl;
}

TEST_F(testQPOasesProblem, testTask)
{
    yarp::sig::Vector q_ref(10, 0.0);
    yarp::sig::Vector q(q_ref.size(), 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();

    wb_sot::tasks::velocity::Postural postural_task(q);
    postural_task.setReference(q_ref);
    postural_task.update(q);

    yarp::sig::Matrix H(q.size(),q.size()); H.eye();
    yarp::sig::Vector g(-1.0*postural_task.getb());

    qpOASES::SQProblem testProblem(q.size(), 0, qpOASES::HST_IDENTITY);
    int nWSR = 132;
    qpOASES::returnValue val = testProblem.init(H.data(), g.data(), NULL, NULL, NULL, NULL,
                                                NULL, nWSR);

    EXPECT_TRUE(val == qpOASES::SUCCESSFUL_RETURN);

    yarp::sig::Vector dq(q.size());
    testProblem.getPrimalSolution(dq.data());
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i] + dq[i], q_ref[i], 1E-12);

    qpOASES::SQProblem testProblem2(q.size(), 0, qpOASES::HST_IDENTITY);
    this->setTestProblem(testProblem2);
    nWSR = 132;
    val = this->_problem.init(H.data(), g.data(), NULL, NULL, NULL, NULL,
                        NULL, nWSR);
    EXPECT_TRUE(val == qpOASES::SUCCESSFUL_RETURN);
    yarp::sig::Vector sol(q.size(), 0.0);
    this->_problem.getPrimalSolution(sol.data());
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i] + dq[i], q_ref[i], 1E-12);
}

TEST_F(testQPOasesTask, testQPOasesTask)
{
    yarp::sig::Matrix A;
    EXPECT_TRUE(A.data() == NULL);

    yarp::sig::Vector q_ref(10, 0.0);
    yarp::sig::Vector q(q_ref.size(), 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();
    std::cout<<"q: "<<q.toString()<<std::endl;

    boost::shared_ptr<wb_sot::tasks::velocity::Postural> postural_task(
                new wb_sot::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    postural_task->update(q);
    std::cout<<"error: "<<postural_task->getb().toString()<<std::endl;

    wb_sot::solvers::QPOasesTask qp_postural_task(postural_task);
    EXPECT_TRUE(qp_postural_task.isQProblemInitialized());

    postural_task->update(q);
    EXPECT_TRUE(qp_postural_task.solve());
    std::cout<<"solution: "<<qp_postural_task.getSolution().toString()<<std::endl;
    q += qp_postural_task.getSolution();

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(q[i], q_ref[i]);

}

using namespace wb_sot::bounds::velocity;
TEST_F(testQPOasesTask, testProblemWithConstraint)
{
        iDynUtils idynutils;
        yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
        yarp::sig::Vector q_ref(q.size(), M_PI);
        idynutils.updateiDyn3Model(q, true);

        boost::shared_ptr<wb_sot::tasks::velocity::Postural> postural_task(
                new wb_sot::tasks::velocity::Postural(q));
        postural_task->setReference(q_ref);
        boost::shared_ptr<JointLimits> joint_limits(
            new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(), idynutils.coman_iDyn3.getJointBoundMin()));
        postural_task->getConstraints().push_back(joint_limits);
        postural_task->setAlpha(0.1);

        wb_sot::solvers::QPOasesTask qp_postural_task(postural_task);
        EXPECT_TRUE(qp_postural_task.isQProblemInitialized());

        yarp::sig::Vector l, u;
        qp_postural_task.getBounds(l, u);
        EXPECT_TRUE(l == idynutils.coman_iDyn3.getJointBoundMin());
        EXPECT_TRUE(u == idynutils.coman_iDyn3.getJointBoundMax());

        yarp::sig::Vector l_old, u_old;
        for(unsigned int i = 0; i < 100; ++i)
        {
            postural_task->update(q);
            qp_postural_task.getBounds(l_old, u_old);
            EXPECT_TRUE(qp_postural_task.solve());
            q += qp_postural_task.getSolution();

            postural_task->update(q);
            qp_postural_task.getBounds(l, u);
            EXPECT_FALSE(l == l_old);
            EXPECT_FALSE(u == u_old);
        }

//        for(unsigned int i = 0; i < q.size(); ++i)
//        {
//            if(q_ref[i] >= idynutils.coman_iDyn3.getJointBoundMax()[i])
//            {
//                std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
//                EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMax()[i], 1E-4);
//            }
//            else if(q_ref[i] <= idynutils.coman_iDyn3.getJointBoundMin()[i])
//            {
//                std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
//                EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMin()[i], 1E-4);
//            }
//            else
//                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
//        }
}

//TEST_F(testQPOases_sot, testContructor1Problem)
//{
//    iDynUtils idynutils;
//    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
//    yarp::sig::Vector q_ref(q.size(), M_PI);
//    idynutils.updateiDyn3Model(q, true);

//    boost::shared_ptr<wb_sot::tasks::velocity::Postural> postural_task(
//            new wb_sot::tasks::velocity::Postural(q));
//    postural_task->setReference(q_ref);
//    boost::shared_ptr<JointLimits> joint_limits(
//        new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(), idynutils.coman_iDyn3.getJointBoundMin()));
//    postural_task->getConstraints().push_back(joint_limits);
//    postural_task->setAlpha(0.1);

//    std::vector<boost::shared_ptr<wb_sot::Task<Matrix, Vector> >> stack_of_tasks;
//    stack_of_tasks.push_back(postural_task);
//    wb_sot::solvers::QPOases_sot sot(stack_of_tasks);

//    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
//    yarp::sig::Vector dq(q.size(), 0.0);
//    for(unsigned int i = 0; i < 100; ++i)
//    {
//        postural_task->update(q);
//        EXPECT_TRUE(sot.solve(dq));
//        q += dq;
//    }

//    for(unsigned int i = 0; i < q.size(); ++i)
//    {
//        if(q_ref[i] >= idynutils.coman_iDyn3.getJointBoundMax()[i])
//        {
//            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
//            EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMax()[i], 1E-4);
//        }
//        else if(q_ref[i] <= idynutils.coman_iDyn3.getJointBoundMin()[i])
//        {
//            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
//            EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMin()[i], 1E-4);
//        }
//        else
//            EXPECT_NEAR( q[i], q_ref[i], 1E-4);

//    }
//}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
