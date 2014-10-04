#include <drc_shared/idynutils.h>
#include <drc_shared/tests_utils.h>
#include <drc_shared/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

bool solveQP(   const yarp::sig::Matrix &J0,
                const yarp::sig::Vector &e0,
                const yarp::sig::Matrix &J1,
                const yarp::sig::Vector &eq,
                qpOASES::HessianType t1HessianType,
                const yarp::sig::Vector &l,
                const yarp::sig::Vector &u,
                const yarp::sig::Vector &q,
                yarp::sig::Vector &dq_ref)
{
    int nj = q.size();

    static bool initial_guess = false;

    static yarp::sig::Vector dq0(nj, 0.0);
    static yarp::sig::Vector dq1(nj, 0.0);;
    static yarp::sig::Vector y0(nj, 0.0);
    static yarp::sig::Vector y1(nj, 0.0);

    static qpOASES::Bounds bounds0;
    static qpOASES::Bounds bounds1;
    static qpOASES::Constraints constraints0;
    static qpOASES::Constraints constraints1;


    /**
      We solve a single QP where the priority between
      different tasks is set by using a weight matrix Q

      min         (Ax - b)'Q(Ax - b)
      subj to     l <=   x <=  u

      QPOASES::Quadratic_program solves by default a quadratic problem in the form
      min         x'Hx + x'g
      subj to  Alb <= Ax <= Aub
                 l <=  x <= u
     **/

    int njTask0 = J0.rows();

    yarp::sig::Matrix H0 = J0.transposed()*J0; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g0 = -1.0*J0.transposed()*e0;

    yarp::sig::Matrix H1 = J1.transposed()*J1; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*eq;

    USING_NAMESPACE_QPOASES

    /** Setting up QProblem object. **/
    Options qpOasesOptionsqp0;
    qpOasesOptionsqp0.printLevel = PL_NONE;
    qpOasesOptionsqp0.setToReliable();
    qpOasesOptionsqp0.enableRegularisation = BT_TRUE;
    qpOasesOptionsqp0.epsRegularisation *= 2E2;
    QProblem qp0( nj, 0, HST_SEMIDEF);
    qp0.setOptions( qpOasesOptionsqp0 );

    Options qpOasesOptionsqp1;
    qpOasesOptionsqp1.printLevel = PL_NONE;
    qpOasesOptionsqp1.setToReliable();
    qpOasesOptionsqp1.enableRegularisation = BT_TRUE;
    qpOasesOptionsqp1.epsRegularisation *= 2E2;
    QProblem qp1( nj, njTask0, t1HessianType);
    qp1.setOptions( qpOasesOptionsqp1 );

    /** Solve zero QP. **/
    int nWSR = 132;
    if(initial_guess==true)
        qp0.init( H0.data(),g0.data(),
                  NULL,
                  l.data(), u.data(),
                  NULL, NULL,
                  nWSR,0,
                  dq0.data(), y0.data(),
                  &bounds0, &constraints0);
    else {
        qp0.init( H0.data(),g0.data(),
                  NULL,
                  l.data(), u.data(),
                  NULL, NULL,
                  nWSR,0);
        std::cout << GREEN << "Not using initial guess" << DEFAULT;
    }

    if(dq0.size() != qp0.getNV()) {
        dq0.resize(qp0.getNV());
        initial_guess = false;
    }
    if(y0.size() != qp0.getNV() + qp0.getNC()) {
        y0.resize(qp0.getNV()+ qp0.getNC());
        initial_guess = false;
    }

    int success0 = qp0.getPrimalSolution( dq0.data() );
    qp0.getDualSolution(y0.data());
    qp0.getBounds(bounds0);
    qp0.getConstraints(constraints0);

    if(success0== RET_QP_NOT_SOLVED ||
      (success0 != RET_QP_SOLVED && success0 != SUCCESSFUL_RETURN))
    {
        std::cout << GREEN <<
                     "ERROR OPTIMIZING ZERO TASK! ERROR #" <<
                     success0 <<
                     "Not using initial guess" << DEFAULT;

        initial_guess = false;
    }
    else
    {
        /** Solve first QP. **/
        yarp::sig::Matrix A1 = J0;
        yarp::sig::Vector b1 = J0*dq0;
        yarp::sig::Vector lA1 = b1;
        yarp::sig::Vector uA1 = b1;

        nWSR = 132;

        if(initial_guess == true)
            qp1.init( H1.data(),g1.data(),
                      A1.data(),
                      l.data(), u.data(),
                      lA1.data(), uA1.data(),
                      nWSR, 0,
                      dq1.data(), y1.data(),
                      &bounds1, &constraints1);
        else
            qp1.init( H1.data(),g1.data(),
                      A1.data(),
                      l.data(), u.data(),
                      lA1.data(), uA1.data(),
                      nWSR, 0);

        if(dq1.size() != qp1.getNV()) {
            dq1.resize(qp1.getNV());
            initial_guess = false;
        }
        if(y1.size() != qp1.getNV() + qp1.getNC()) {
            y1.resize(qp1.getNV() + qp1.getNC());
            initial_guess = false;
        }

        int success1 = qp1.getPrimalSolution( dq1.data() );
        qp1.getDualSolution(y1.data());
        qp1.getBounds(bounds1);
        qp1.getConstraints(constraints1);

        if(success1 == RET_QP_NOT_SOLVED ||
          (success1 != RET_QP_SOLVED && success1 != SUCCESSFUL_RETURN))
        {
            std::cout << GREEN <<
                         "ERROR OPTIMIZING POSTURE TASK! ERROR #" <<
                         success1 << DEFAULT;
            initial_guess = false;
        }
        else
        {
            dq_ref = dq1;
            initial_guess = true;
            return true;
       }
    }
    return false;
}

bool solveQPrefactor(   const yarp::sig::Matrix &J0,
                        const yarp::sig::Vector &e0,
                        const yarp::sig::Matrix &J1,
                        const yarp::sig::Vector &eq,
                        OpenSoT::HessianType t1HessianType,
                        const yarp::sig::Vector &u,
                        const yarp::sig::Vector &l,
                        const yarp::sig::Vector &q,
                        yarp::sig::Vector &dq_ref)
{
    int nj = q.size();

    int njTask0 = J0.rows();

    yarp::sig::Matrix H0 = J0.transposed()*J0; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g0 = -1.0*J0.transposed()*e0;

    yarp::sig::Matrix H1 = J1.transposed()*J1; // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*eq;

    yarp::sig::Matrix A0(0,nj);
    yarp::sig::Vector lA0(0), uA0(0);

    USING_NAMESPACE_QPOASES

    static OpenSoT::solvers::QPOasesProblem qp0(nj, 0, OpenSoT::HST_SEMIDEF);
    qp0.setnWSR(127);
    static bool result0 = false;
    if(!qp0.isQProblemInitialized())
        result0 = qp0.initProblem(H0, g0, A0, lA0, uA0, l, u);
    else
    {
        qp0.updateProblem(H0, g0, A0, lA0, uA0, l, u);
        result0 = qp0.solve();
    }

    if(result0)
    {
        yarp::sig::Vector dq0 = qp0.getSolution();
        yarp::sig::Matrix A1 = J0;
        yarp::sig::Vector b1 = J0*dq0;
        yarp::sig::Vector lA1 = b1;
        yarp::sig::Vector uA1 = b1;

        static OpenSoT::solvers::QPOasesProblem qp1(nj, njTask0, t1HessianType);
        qp1.setnWSR(127);
        static bool result1 = false;
        if(!qp1.isQProblemInitialized())
            result1 = qp1.initProblem(H1, g1, A1, lA1, uA1, l, u);
        else
        {
            qp1.updateProblem(H1, g1, A1, lA1, uA1, l, u);
            result1 = qp1.solve();
        }
        if(result1)
        {
            dq_ref = qp1.getSolution();
            return true;
        }
        else
        {
            std::cout << GREEN << "ERROR OPTIMIZING POSTURE TASK" << DEFAULT;
            return false;
        }
    }
    else
    {
        std::cout << GREEN << "ERROR OPTIMIZING CARTESIAN TASK" << DEFAULT;
        return false;
    }
}

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
        public OpenSoT::solvers::QPOasesProblem
{
protected:

    testQPOasesProblem()
    {

    }

    void setTestProblem(const boost::shared_ptr<qpOASES::SQProblem> &problem)
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

    boost::shared_ptr<qpOASES::SQProblem> testProblem(
                new qpOASES::SQProblem(x.size(), sp.A.rows(), sp.ht) );
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

    boost::shared_ptr<qpOASES::SQProblem> testProblem(
                new qpOASES::SQProblem(x.size(), sp.A.rows(), sp.ht) );
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


    boost::shared_ptr<qpOASES::SQProblem> testProblem(
                new qpOASES::SQProblem(x.size(), A.rows(), ht) );
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

    OpenSoT::tasks::velocity::Postural postural_task(q);
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

    boost::shared_ptr<qpOASES::SQProblem> testProblem2(
                new qpOASES::SQProblem(q.size(), 0, qpOASES::HST_IDENTITY) );
    this->setTestProblem(testProblem2);
    nWSR = 132;
    val = this->_problem->init(H.data(), g.data(), NULL, NULL, NULL, NULL,
                        NULL, nWSR);
    EXPECT_TRUE(val == qpOASES::SUCCESSFUL_RETURN);
    yarp::sig::Vector sol(q.size(), 0.0);
    this->_problem->getPrimalSolution(sol.data());
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

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    postural_task->update(q);
    std::cout<<"error: "<<postural_task->getb().toString()<<std::endl;

    OpenSoT::solvers::QPOasesTask qp_postural_task(postural_task);
    EXPECT_TRUE(qp_postural_task.isQProblemInitialized());

    postural_task->update(q);
    EXPECT_TRUE(qp_postural_task.solve());
    std::cout<<"solution: "<<qp_postural_task.getSolution().toString()<<std::endl;
    q += qp_postural_task.getSolution();

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(q[i], q_ref[i]);

}

using namespace OpenSoT::constraints::velocity;
TEST_F(testQPOasesTask, testProblemWithConstraint)
{
        iDynUtils idynutils;
        yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
        yarp::sig::Vector q_ref(q.size(), M_PI);
        idynutils.updateiDyn3Model(q, true);

        boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
        postural_task->setReference(q_ref);
        boost::shared_ptr<JointLimits> joint_limits(
            new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(), idynutils.coman_iDyn3.getJointBoundMin()));
        postural_task->getConstraints().push_back(joint_limits);
        postural_task->setLambda(0.1);

        OpenSoT::solvers::QPOasesTask qp_postural_task(postural_task);
        EXPECT_TRUE(qp_postural_task.isQProblemInitialized());

        yarp::sig::Vector l_old, u_old;
        qp_postural_task.getBounds(l_old, u_old);
        EXPECT_TRUE(l_old == idynutils.coman_iDyn3.getJointBoundMin());
        EXPECT_TRUE(u_old == idynutils.coman_iDyn3.getJointBoundMax());

        yarp::sig::Vector l, u;
        for(unsigned int i = 0; i < 100; ++i)
        {
            postural_task->update(q);
            EXPECT_TRUE(qp_postural_task.solve());
            qp_postural_task.getBounds(l, u);
            q += qp_postural_task.getSolution();

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 0; i < q.size(); ++i)
        {
            if(q_ref[i] >= idynutils.coman_iDyn3.getJointBoundMax()[i])
            {
                std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMax()[i], 1E-4);
            }
            else if(q_ref[i] <= idynutils.coman_iDyn3.getJointBoundMin()[i])
            {
                std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMin()[i], 1E-4);
            }
            else
                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
        }
}

TEST_F(testQPOases_sot, testContructor1Problem)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector q_ref(q.size(), M_PI);
    idynutils.updateiDyn3Model(q, true);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(), idynutils.coman_iDyn3.getJointBoundMin()));
    postural_task->setLambda(0.1);

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> bounds_list;
    bounds_list.push_back(joint_limits);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 100; ++i)
    {
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot.solve(dq));
        q += dq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
    {
        if(q_ref[i] >= idynutils.coman_iDyn3.getJointBoundMax()[i])
        {
            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMax()[i], 1E-4);
        }
        else if(q_ref[i] <= idynutils.coman_iDyn3.getJointBoundMin()[i])
        {
            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], idynutils.coman_iDyn3.getJointBoundMin()[i], 1E-4);
        }
        else
            EXPECT_NEAR( q[i], q_ref[i], 1E-4);

    }
}

TEST_F(testQPOasesTask, testCartesian)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Matrix T = idynutils.coman_iDyn3.getPosition(
                          idynutils.coman_iDyn3.getLinkIndex("Waist"),
                          idynutils.coman_iDyn3.getLinkIndex("l_wrist"));

    //2 Tasks: Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, idynutils,
                "l_wrist", "Waist"));


    yarp::sig::Matrix T_ref = T;
    T_ref(0,3) = T_ref(0,3) + 0.02;

    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

    OpenSoT::solvers::QPOasesTask cartesian_qp(cartesian_task);

    for(unsigned int i = 0; i < 100; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        cartesian_task->update(q);

        ASSERT_TRUE(cartesian_qp.solve());
        q += cartesian_qp.getSolution();
    }

    T = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex("Waist"),
                idynutils.coman_iDyn3.getLinkIndex("l_wrist"));

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T(i,3), T_ref(i,3), 1E-4);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-4);
}

TEST_F(testQPOases_sot, testContructor2Problems)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    yarp::sig::Vector torso(idynutils.torso.getNrOfDOFs(), 0.0);
    torso[0] = tests_utils::getRandomAngle(-20.0*M_PI/180.0, 20.0*M_PI/180.0);
    torso[1] = tests_utils::getRandomAngle(0.0, 45.0*M_PI/180.0);
    torso[2] = tests_utils::getRandomAngle(-30.0*M_PI/180.0, 30.0*M_PI/180.0);
    idynutils.fromRobotToIDyn(torso, q, idynutils.torso);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Matrix T_init = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex("Waist"),
                idynutils.coman_iDyn3.getLinkIndex("l_wrist"));
    std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);


    //2 Tasks: Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::l_wrist", q, idynutils,
                                                       "l_wrist", "Waist"));
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));

    postural_task->setReference(q);
    cartesian_task->setReference(T_init);

    int t = 50;
    //Constraints set to the Cartesian Task
    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(),
                           idynutils.coman_iDyn3.getJointBoundMin()));
    joint_limits->setBoundScaling((double)(1.0/t));

    boost::shared_ptr<VelocityLimits> joint_velocity_limits(
                new VelocityLimits(0.3, (double)(1.0/t), q.size()));

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    //Create the SoT
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    std::cout<<"Initial Position Error: "<<cartesian_task->positionError.toString()<<std::endl;
    std::cout<<"Initial Orientation Error: "<<cartesian_task->orientationError.toString()<<std::endl;

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);


    KDL::Frame T_ref_kdl;
    T_ref_kdl.p[0] = 0.283; T_ref_kdl.p[1] = 0.156; T_ref_kdl.p[2] = 0.499;
    T_ref_kdl.M = T_ref_kdl.M.Quaternion(0.0, 0.975, 0.0, -0.221);
    yarp::sig::Matrix T_ref;
    cartesian_utils::fromKDLFrameToYARPMatrix(T_ref_kdl, T_ref);

    cartesian_task->setReference(T_ref);

    //Solve SoT
    idynutils.updateiDyn3Model(q, true);


    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 10*t; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        cartesian_task->update(q);
        postural_task->update(q);
        joint_constraints->update(q);

        ASSERT_TRUE(sot.solve(dq));
        q += dq;
    }

    idynutils.updateiDyn3Model(q);
//    std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);
    yarp::sig::Matrix T = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex("Waist"),
                idynutils.coman_iDyn3.getLinkIndex("l_wrist"));
//    std::cout<<"FINAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T);
//    std::cout<<"DESIRED CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_ref);


    KDL::Frame T_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T, T_kdl);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.p[i], T_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.M(i,j), T_ref_kdl.M(i,j), 1E-2);


}


TEST_F(testQPOases_sot, test2ProblemsWithQPSolve)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Matrix T_init = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex("Waist"),
                idynutils.coman_iDyn3.getLinkIndex("r_wrist"));

    //2 Tasks: Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::r_wrist", q, idynutils,
                                                       "r_wrist", "Waist"));
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));

    KDL::Frame T_ref_kdl;
    yarp::sig::Matrix T_ref;

    T_ref = T_init;
    T_ref(2,3) = T_ref(2,3) + 0.05;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_ref, T_ref_kdl);

    cartesian_task->setReference(T_ref);
    cartesian_task->setLambda(0.1);
    cartesian_task->setOrientationErrorGain(1.0);
    postural_task->setReference(q);
    postural_task->setLambda(0.1);


    int t = 50;
    std::list< OpenSoT::constraints::Aggregated::ConstraintPtr> constraints_list;

    //Constraints set to the Cartesian Task
    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(),
                           idynutils.coman_iDyn3.getJointBoundMin()));
    joint_limits->setBoundScaling((double)(1.0/t));
    constraints_list.push_back(joint_limits);

    boost::shared_ptr<VelocityLimits> joint_velocity_limits(
                new VelocityLimits(0.3, (double)(1.0/t), q.size()));
    constraints_list.push_back(joint_velocity_limits);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> aggregated_bounds(
                new OpenSoT::constraints::Aggregated(constraints_list,q.size()));

    //Solve SoT
    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 10*t; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        cartesian_task->update(q);
        postural_task->update(q);
        aggregated_bounds->update(q);

        ASSERT_TRUE(solveQP(cartesian_task->getA(), cartesian_task->getb(),
                postural_task->getA(), postural_task->getb(),
                qpOASES::HST_SEMIDEF,
                aggregated_bounds->getLowerBound(),
                aggregated_bounds->getUpperBound(),
                q, dq));

        q += dq;
////////////////////////////////////////////////////////////////
    }

    idynutils.updateiDyn3Model(q);
    //std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);
    yarp::sig::Matrix T = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex("Waist"),
                idynutils.coman_iDyn3.getLinkIndex("r_wrist"));
//    std::cout<<"FINAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T);
//    std::cout<<"DESIRED CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_ref);


    KDL::Frame T_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T, T_kdl);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.p[i], T_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.M(i,j), T_ref_kdl.M(i,j), 1E-2);
}

TEST_F(testQPOases_sot, testUpTo4Problems)
{
    srand(time(NULL));
    int number_of_tasks = rand() % 4;  //number between 1 and 3

    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Vector q_ref(q.size(), 0.0);
    q_ref = q;

    //3 Tasks: CoM & Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::CoM> com_task(
                new OpenSoT::tasks::velocity::CoM(q));

    std::string ee1 = "r_wrist";
    std::string ee2 = "l_wrist";
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::"+ee1, q, idynutils,
                                                       ee1, "world"));
    cartesian_task->setLambda(1.0);

    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task2(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::"+ee2, q, idynutils,
                                                       ee2, "world"));
    cartesian_task2->setLambda(1.0);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setLambda(1.0);

    //Bounds
    int t = 50;
    std::list< OpenSoT::constraints::Aggregated::ConstraintPtr> constraints_list;

    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.coman_iDyn3.getJointBoundMax(),
                           idynutils.coman_iDyn3.getJointBoundMin()));
    joint_limits->setBoundScaling((double)(1.0));
    constraints_list.push_back(joint_limits);

    boost::shared_ptr<VelocityLimits> joint_velocity_limits(
                new VelocityLimits(0.3, (double)(1.0/t), q.size()));
    constraints_list.push_back(joint_velocity_limits);


    boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints(
                new OpenSoT::constraints::Aggregated(constraints_list, q.size()));

    yarp::sig::Matrix T_arm_init = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex(ee1));
    yarp::sig::Matrix T_arm_ref = T_arm_init;
    T_arm_ref(2,3) += 0.05;
    KDL::Frame T_arm_ref_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm_ref, T_arm_ref_kdl);

    yarp::sig::Matrix T_arm2_init = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex(ee2));
    yarp::sig::Matrix T_arm2_ref = T_arm2_init;
    T_arm2_ref(0,3) += 0.05;
    KDL::Frame T_arm2_ref_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2_ref, T_arm2_ref_kdl);


    yarp::sig::Vector T_com_p_init = idynutils.coman_iDyn3.getCOM("",
                com_task->getLinkWRTCoMIsSpecified());
    yarp::sig::Vector T_com_p_ref = T_com_p_init;
    T_com_p_ref[1] += 0.1;
    yarp::sig::Matrix T_com_ref(4,4); T_com_ref.eye();
    T_com_ref(0,3) = T_com_p_ref[0];
    T_com_ref(1,3) = T_com_p_ref[1];
    T_com_ref(2,3) = T_com_p_ref[2];
    KDL::Frame T_com_ref_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_com_ref, T_com_ref_kdl);

    com_task->setReference(T_com_p_ref);
    cartesian_task->setReference(T_arm_ref);
    cartesian_task2->setReference(T_arm2_ref);
    postural_task->setReference(q_ref);


    //Create the SoT
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    if(number_of_tasks >= 1)
        stack_of_tasks.push_back(com_task);
    if(number_of_tasks >= 2)
        stack_of_tasks.push_back(cartesian_task);
    if(number_of_tasks >= 3)
        stack_of_tasks.push_back(cartesian_task2);

    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    yarp::sig::Vector dq(q.size(), 0.0);
    double acc = 0.0;
    int s = 10;
    for(unsigned int i = 0; i < s*t; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        com_task->update(q);
        cartesian_task->update(q);
        cartesian_task2->update(q);
        postural_task->update(q);
        joint_constraints->update(q);

        double tic = yarp::os::Time::now();
        ASSERT_TRUE(sot.solve(dq));
        double toc = yarp::os::Time::now();
        acc += toc - tic;

        //std::cout<<"dq: "<<dq.toString()<<std::endl;
        q += dq;
    }
    std::cout<<"Medium Time to Solve sot "<<acc/(double)(s*t)<<"[s]"<<std::endl;


    yarp::sig::Matrix T_arm = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex(ee1));
    KDL::Frame T_arm_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm, T_arm_kdl);

    yarp::sig::Matrix T_arm2 = idynutils.coman_iDyn3.getPosition(
                idynutils.coman_iDyn3.getLinkIndex(ee2));
    KDL::Frame T_arm2_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2, T_arm2_kdl);

    yarp::sig::Vector T_com_p = idynutils.coman_iDyn3.getCOM("",
                com_task->getLinkWRTCoMIsSpecified());
    yarp::sig::Matrix T_com(4,4); T_com.eye();
    T_com(0,3) = T_com_p[0];
    T_com(1,3) = T_com_p[1];
    T_com(2,3) = T_com_p[2];
    KDL::Frame T_com_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_com, T_com_kdl);

    if(number_of_tasks >= 1)
    {
        yarp::sig::Matrix T_com_init(4,4);
        T_com_init(0,3) = T_com_p_init[0];
        T_com_init(1,3) = T_com_p_init[1];
        T_com_init(2,3) = T_com_p_init[2];
        std::cout<<GREEN<<"CoM Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_com_init);
        std::cout<<GREEN<<"CoM Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_com_ref_kdl);
        std::cout<<GREEN<<"CoM Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_com_kdl);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_com_kdl.p[i], T_com_ref_kdl.p[i], 1E-3)<<"For i: "<<i<<std::endl;
    }


    if(number_of_tasks >= 2)
    {
        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_arm_init);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm_ref_kdl);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm_kdl);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_arm_kdl.p[i], T_arm_ref_kdl.p[i], 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_arm_kdl.M(i,j), T_arm_ref_kdl.M(i,j), 1E-2);
    }

    if(number_of_tasks >= 3)
    {
        std::cout<<GREEN<<"Arm2 Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_arm2_init);
        std::cout<<GREEN<<"Arm2 Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm2_ref_kdl);
        std::cout<<GREEN<<"Arm2 Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm2_kdl);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_arm2_kdl.p[i], T_arm2_ref_kdl.p[i], 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_arm2_kdl.M(i,j), T_arm2_ref_kdl.M(i,j), 1E-2);
    }

    if(number_of_tasks == 0)
    {
        for(unsigned int i = 0; i < q.size(); ++i)
            EXPECT_DOUBLE_EQ(q[i], q_ref[i]);
    }



}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
