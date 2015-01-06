#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class old_gravity_gradient
{
public:
    iDynUtils idynutils;

    old_gravity_gradient(){}

    yarp::sig::Vector computeMinEffort(const yarp::sig::Vector& q)
    {
        yarp::sig::Matrix W(idynutils.iDyn3_model.getJointTorqueMax().size(), idynutils.iDyn3_model.getJointTorqueMax().size());
        W.eye();

        for(unsigned int i = 0; i < idynutils.iDyn3_model.getJointTorqueMax().size(); ++i)
            W(i,i) = 1.0 / (idynutils.iDyn3_model.getJointTorqueMax()[i]*idynutils.iDyn3_model.getJointTorqueMax()[i]);

        return -1.0 * getGravityCompensationGradient(W, q);
    }

    yarp::sig::Vector getGravityCompensationGradient(const yarp::sig::Matrix& W, const yarp::sig::Vector& q)
    {

        /// cost function is tau_g^t*tau_g
        yarp::sig::Vector gradient(q.size(),0.0);
        yarp::sig::Vector deltas(q.size(),0.0);
        const double h = 1E-3;
        for(unsigned int i = 0; i < gradient.size(); ++i)
        {
            // forward method gradient computation, milligrad
            deltas[i] = h;
            yarp::sig::Vector tau_gravity_q_a = getGravityCompensationTorque(q+deltas);
            yarp::sig::Vector tau_gravity_q_b = getGravityCompensationTorque(q-deltas);

            double C_g_q_a = yarp::math::dot(tau_gravity_q_a, W*tau_gravity_q_a);
            double C_g_q_b = yarp::math::dot(tau_gravity_q_b, W*tau_gravity_q_b);
            gradient[i] = (C_g_q_a - C_g_q_b)/(2.0*h);
            deltas[i] = 0.0;
        }

        return gradient;
    }

    yarp::sig::Vector getGravityCompensationTorque(const yarp::sig::Vector q)
    {
        static yarp::sig::Vector zeroes(q.size(),0.0);
        static yarp::sig::Vector tau(q.size(),0.0);


        idynutils.updateiDyn3Model(q,zeroes,zeroes, true);

        idynutils.iDyn3_model.dynamicRNEA();
        tau = idynutils.iDyn3_model.getTorques();

        return tau;
    }

};

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
    static bool isQProblemInitialized0 = false;
    if(!isQProblemInitialized0){
        result0 = qp0.initProblem(H0, g0, A0, lA0, uA0, l, u);
        isQProblemInitialized0 = true;}
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
        static bool isQProblemInitialized1 = false;
        if(!isQProblemInitialized1){
            result1 = qp1.initProblem(H1, g1, A1, lA1, uA1, l, u);
            isQProblemInitialized1 = true;}
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

class testQPOasesProblem: public ::testing::Test
{
protected:

    testQPOasesProblem()
    {

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


yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
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
    return q;
}


/**
 * @brief TEST_F testSimpleProblem test solution of a simple CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testSimpleProblem)
{
    yarp::sig::Vector x(2);
    simpleProblem sp;

    OpenSoT::solvers::QPOasesProblem testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);

    testProblem.initProblem(sp.H, sp.g, sp.A, sp.lA, sp.uA, sp.l, sp.u);

    EXPECT_TRUE(testProblem.solve());
    yarp::sig::Vector s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblem.solve());

        yarp::sig::Vector s = testProblem.getSolution();
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

    OpenSoT::solvers::QPOasesProblem testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);

    testProblem.initProblem(sp.H, sp.g, sp.A, sp.lA, sp.uA, sp.l, sp.u);

    EXPECT_TRUE(testProblem.solve());
    yarp::sig::Vector s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    sp.g[0] = -1.0; sp.g[1] = 1.0;
    testProblem.updateTask(sp.H, sp.g);
    EXPECT_TRUE(testProblem.solve());

    s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);
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

    OpenSoT::solvers::QPOasesProblem qp_postural_problem(postural_task->getXSize(), 0,
                                                         postural_task->getHessianAtype());

    EXPECT_TRUE(qp_postural_problem.initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                yarp::sig::Matrix(), yarp::sig::Vector(), yarp::sig::Vector(),
                                                yarp::sig::Vector(), yarp::sig::Vector()));
    std::cout<<"solution: "<<qp_postural_problem.getSolution().toString()<<std::endl;
    q += qp_postural_problem.getSolution();

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(q[i], q_ref[i]);

}

using namespace OpenSoT::constraints::velocity;
TEST_F(testQPOasesTask, testProblemWithConstraint)
{
        iDynUtils idynutils;
        yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
        yarp::sig::Vector q_ref(q.size(), M_PI);
        idynutils.updateiDyn3Model(q, true);

        boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
        postural_task->setReference(q_ref);
        boost::shared_ptr<JointLimits> joint_limits(
            new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(), idynutils.iDyn3_model.getJointBoundMin()));
        postural_task->getConstraints().push_back(joint_limits);
        postural_task->setLambda(0.1);

        OpenSoT::solvers::QPOasesProblem qp_postural_problem(postural_task->getXSize(), 0,
                                                             postural_task->getHessianAtype());
        std::list< boost::shared_ptr<OpenSoT::Constraint< yarp::sig::Matrix, yarp::sig::Vector >>> constraint_list =
                postural_task->getConstraints();
        boost::shared_ptr<OpenSoT::Constraint< yarp::sig::Matrix, yarp::sig::Vector >> constraint = constraint_list.front();
        EXPECT_TRUE(qp_postural_problem.initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                    yarp::sig::Matrix(), yarp::sig::Vector(), yarp::sig::Vector(),
                                                    constraint->getLowerBound(), constraint->getUpperBound()));

        yarp::sig::Vector l_old = qp_postural_problem.getl();
        yarp::sig::Vector u_old = qp_postural_problem.getu();

        EXPECT_TRUE(l_old == idynutils.iDyn3_model.getJointBoundMin());
        EXPECT_TRUE(u_old == idynutils.iDyn3_model.getJointBoundMax());

        yarp::sig::Vector l, u;
        for(unsigned int i = 0; i < 100; ++i)
        {
            postural_task->update(q);

            qp_postural_problem.updateProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                              yarp::sig::Matrix(), yarp::sig::Vector(), yarp::sig::Vector(),
                                              constraint->getLowerBound(), constraint->getUpperBound());
            EXPECT_TRUE(qp_postural_problem.solve());
            l = qp_postural_problem.getl();
            u = qp_postural_problem.getu();
            q += qp_postural_problem.getSolution();

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 0; i < q.size(); ++i)
        {
            if(q_ref[i] >= idynutils.iDyn3_model.getJointBoundMax()[i])
            {
                std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], idynutils.iDyn3_model.getJointBoundMax()[i], 1E-4);
            }
            else if(q_ref[i] <= idynutils.iDyn3_model.getJointBoundMin()[i])
            {
                std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], idynutils.iDyn3_model.getJointBoundMin()[i], 1E-4);
            }
            else
                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
        }
}

TEST_F(testQPOases_sot, testContructor1Problem)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector q_ref(q.size(), M_PI);
    idynutils.updateiDyn3Model(q, true);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(), idynutils.iDyn3_model.getJointBoundMin()));
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
        if(q_ref[i] >= idynutils.iDyn3_model.getJointBoundMax()[i])
        {
            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], idynutils.iDyn3_model.getJointBoundMax()[i], 1E-4);
        }
        else if(q_ref[i] <= idynutils.iDyn3_model.getJointBoundMin()[i])
        {
            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], idynutils.iDyn3_model.getJointBoundMin()[i], 1E-4);
        }
        else
            EXPECT_NEAR( q[i], q_ref[i], 1E-4);

    }
}

TEST_F(testQPOasesTask, testCoMTask)
{
    iDynUtils idynutils;
    idynutils.iDyn3_model.setFloatingBaseLink(idynutils.left_leg.index);
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);

    boost::shared_ptr<OpenSoT::tasks::velocity::CoM> com_task(
                new OpenSoT::tasks::velocity::CoM(q, idynutils));

    boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity> com_vel_constr(
                new OpenSoT::constraints::velocity::CoMVelocity(
                    yarp::sig::Vector(3,0.06),1.0,q,idynutils));
    com_task->getConstraints().push_back(com_vel_constr);

    std::list< boost::shared_ptr<OpenSoT::Constraint< yarp::sig::Matrix, yarp::sig::Vector >>> constraint_list =
            com_task->getConstraints();
    boost::shared_ptr<OpenSoT::Constraint< yarp::sig::Matrix, yarp::sig::Vector >> constraint = constraint_list.front();

    OpenSoT::solvers::QPOasesProblem qp_CoM_problem(com_task->getXSize(), constraint->getAineq().rows(),
                                                    com_task->getHessianAtype());
    ASSERT_TRUE(qp_CoM_problem.initProblem(com_task->getA().transposed()*com_task->getA(), -1.0*com_task->getA().transposed()*com_task->getb(),
                                                constraint->getAineq(), constraint->getbLowerBound(), constraint->getbUpperBound(),
                                                yarp::sig::Vector(), yarp::sig::Vector()));


    yarp::sig::Vector com_i = com_task->getActualPosition();
    yarp::sig::Vector com_f = com_i;
    com_f[0] += 0.05;
    com_f[1] += 0.05;
    com_f[2] -= 0.05;
    com_task->setReference(com_f);


    for(unsigned int i = 0; i < 100; ++i)
    {
        idynutils.updateiDyn3Model(q,true);
        com_task->update(q);

        qp_CoM_problem.updateProblem(com_task->getA().transposed()*com_task->getA(), -1.0*com_task->getA().transposed()*com_task->getb(),
                                          constraint->getAineq(), constraint->getbLowerBound(), constraint->getbUpperBound(),
                                          yarp::sig::Vector(), yarp::sig::Vector());

        ASSERT_TRUE(qp_CoM_problem.solve());
        yarp::sig::Vector dq = qp_CoM_problem.getSolution();
        q += dq;
    }


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR( com_task->getb()[i], 0.0, 1E-4);

}

TEST_F(testQPOasesTask, testCartesian)
{
    iDynUtils idynutils;
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Matrix T = idynutils.iDyn3_model.getPosition(
                          idynutils.iDyn3_model.getLinkIndex("Waist"),
                          idynutils.iDyn3_model.getLinkIndex("l_wrist"));

    //2 Tasks: Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, idynutils,
                "l_wrist", "Waist"));


    yarp::sig::Matrix T_ref = T;
    T_ref(0,3) = T_ref(0,3) + 0.02;

    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

    OpenSoT::solvers::QPOasesProblem qp_cartesian_problem(cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype());
    ASSERT_TRUE(qp_cartesian_problem.initProblem(cartesian_task->getA().transposed()*cartesian_task->getA(), -1.0*cartesian_task->getA().transposed()*cartesian_task->getb(),
                                                yarp::sig::Matrix(), yarp::sig::Vector(), yarp::sig::Vector(),
                                                yarp::sig::Vector(), yarp::sig::Vector()));

    for(unsigned int i = 0; i < 100; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        cartesian_task->update(q);
        qp_cartesian_problem.updateTask(cartesian_task->getA().transposed()*cartesian_task->getA(), -1.0*cartesian_task->getA().transposed()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem.solve());
        q += qp_cartesian_problem.getSolution();
    }

    T = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex("l_wrist"));

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T(i,3), T_ref(i,3), 1E-4);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-4);
}

TEST_F(testQPOases_sot, testContructor2Problems)
{
    iDynUtils idynutils;
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    yarp::sig::Vector torso(idynutils.torso.getNrOfDOFs(), 0.0);
    torso[0] = tests_utils::getRandomAngle(-20.0*M_PI/180.0, 20.0*M_PI/180.0);
    torso[1] = tests_utils::getRandomAngle(0.0, 45.0*M_PI/180.0);
    torso[2] = tests_utils::getRandomAngle(-30.0*M_PI/180.0, 30.0*M_PI/180.0);
    idynutils.fromRobotToIDyn(torso, q, idynutils.torso);
    idynutils.updateiDyn3Model(q, true);

    yarp::sig::Matrix T_init = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex("l_wrist"));
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
        new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(),
                           idynutils.iDyn3_model.getJointBoundMin()));
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
        //std::cout<<"Solution: ["<<dq.toString()<<"]"<<std::endl;
        q += dq;
    }

    idynutils.updateiDyn3Model(q);
    std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);
    yarp::sig::Matrix T = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T);
    std::cout<<"DESIRED CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_ref);


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
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);

    std::string ee = "l_wrist"; //r_wrist

    yarp::sig::Matrix T_init = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex(ee));

    //2 Tasks: Cartesian & Postural
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::"+ee, q, idynutils,
                                                       ee, "Waist"));
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
        new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(),
                           idynutils.iDyn3_model.getJointBoundMin()));
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
    yarp::sig::Matrix T = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex(ee));
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
    for(unsigned int jj = 0; jj < 4; ++jj)
    {
        //srand(time(NULL));
        int number_of_tasks = jj;//rand() % 4;  //number between 1 and 3

        iDynUtils idynutils;
        iDynUtils idynutils_com;
        idynutils_com.iDyn3_model.setFloatingBaseLink(idynutils_com.left_leg.index);
        yarp::sig::Vector q = getGoodInitialPosition(idynutils);
        idynutils.updateiDyn3Model(q, true);
        idynutils_com.updateiDyn3Model(q, true);

        yarp::sig::Vector q_ref(q.size(), 0.0);
        q_ref = q;

        //3 Tasks: CoM & Cartesian & Postural
        boost::shared_ptr<OpenSoT::tasks::velocity::CoM> com_task(
                    new OpenSoT::tasks::velocity::CoM(q, idynutils_com));
        com_task->setLambda(1.0);

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
            new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(),
                               idynutils.iDyn3_model.getJointBoundMin()));
        joint_limits->setBoundScaling((double)(1.0));
        constraints_list.push_back(joint_limits);

        boost::shared_ptr<VelocityLimits> joint_velocity_limits(
                    new VelocityLimits(0.3, (double)(1.0/t), q.size()));
        constraints_list.push_back(joint_velocity_limits);


        boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints(
                    new OpenSoT::constraints::Aggregated(constraints_list, q.size()));

        yarp::sig::Matrix T_arm_init = idynutils.iDyn3_model.getPosition(
                    idynutils.iDyn3_model.getLinkIndex(ee1));
        yarp::sig::Matrix T_arm_ref = T_arm_init;
        T_arm_ref(2,3) += 0.05;
        KDL::Frame T_arm_ref_kdl;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm_ref, T_arm_ref_kdl);

        yarp::sig::Matrix T_arm2_init = idynutils.iDyn3_model.getPosition(
                    idynutils.iDyn3_model.getLinkIndex(ee2));
        yarp::sig::Matrix T_arm2_ref = T_arm2_init;
        T_arm2_ref(0,3) += 0.05;
        KDL::Frame T_arm2_ref_kdl;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2_ref, T_arm2_ref_kdl);


        yarp::sig::Vector T_com_p_init = idynutils.iDyn3_model.getCOM();
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
        OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints, 2E2);

        yarp::sig::Vector dq(q.size(), 0.0);
        double acc = 0.0;
        int s = 10;
        for(unsigned int i = 0; i < s*t; ++i)
        {
            idynutils.updateiDyn3Model(q, true);
            idynutils_com.updateiDyn3Model(q,true);

            com_task->update(q);
            cartesian_task->update(q);
            cartesian_task2->update(q);
            postural_task->update(q);
            joint_constraints->update(q);

            double tic = yarp::os::Time::now();
            sot.solve(dq);
            double toc = yarp::os::Time::now();
            acc += toc - tic;

            //std::cout<<"dq: "<<dq.toString()<<std::endl;
            q += dq;
        }
        std::cout<<"Medium Time to Solve sot "<<acc/(double)(s*t)<<"[s]"<<std::endl;


        yarp::sig::Matrix T_arm = idynutils.iDyn3_model.getPosition(
                    idynutils.iDyn3_model.getLinkIndex(ee1));
        KDL::Frame T_arm_kdl;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm, T_arm_kdl);

        yarp::sig::Matrix T_arm2 = idynutils.iDyn3_model.getPosition(
                    idynutils.iDyn3_model.getLinkIndex(ee2));
        KDL::Frame T_arm2_kdl;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2, T_arm2_kdl);

        yarp::sig::Vector T_com_p = idynutils_com.iDyn3_model.getCOM();
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

TEST_F(testQPOases_sot, testContructor1ProblemAggregated)
{
    int n_dofs = 5;
    yarp::sig::Vector q(n_dofs, 0.0);
    yarp::sig::Vector q_ref(q.size(), M_PI);

    yarp::sig::Vector q2(n_dofs, 0.0);
    yarp::sig::Vector q_ref2(q2.size(), M_PI);


    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task2(
            new OpenSoT::tasks::velocity::Postural(q2));
    postural_task2->setReference(q_ref2);
    std::list<boost::shared_ptr<OpenSoT::Task<Matrix, Vector>>> task_list;
    task_list.push_back(postural_task2);
    boost::shared_ptr<OpenSoT::tasks::Aggregated> joint_space_task(
                new OpenSoT::tasks::Aggregated(task_list, q2.size()));


    boost::shared_ptr<OpenSoT::constraints::velocity::VelocityLimits> joint_vel_limits(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q.size()));
    boost::shared_ptr<OpenSoT::constraints::velocity::VelocityLimits> joint_vel_limits2(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q2.size()));

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> bounds_list;
    bounds_list.push_back(joint_vel_limits);
    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> bounds_list2;
    bounds_list2.push_back(joint_vel_limits2);


    boost::shared_ptr<OpenSoT::constraints::Aggregated> bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));
    boost::shared_ptr<OpenSoT::constraints::Aggregated> bounds2(
                new OpenSoT::constraints::Aggregated(bounds_list2, q2.size()));

//1. Here we use postural_task
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot.solve(dq));
        q += dq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i], q_ref[i], 1E-4);


////2. Here we use joint_space_task
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks2;
    stack_of_tasks2.push_back(joint_space_task);
    OpenSoT::solvers::QPOases_sot sot2(stack_of_tasks2, bounds2);

    EXPECT_TRUE(sot2.getNumberOfTasks() == 1);
    yarp::sig::Vector dq2(q2.size(), 0.0);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        joint_space_task->update(q2);
        bounds2->update(q2);
        EXPECT_TRUE(sot2.solve(dq2));
        q2 += dq2;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q2[i], q_ref2[i], 1E-4);

}

TEST_F(testQPOases_sot, testMinEffort)
{
    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[5] = 45.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.updateiDyn3Model(q,true);

    boost::shared_ptr<OpenSoT::tasks::velocity::MinimumEffort> min_effort_task(
            new OpenSoT::tasks::velocity::MinimumEffort(q, idynutils));

    std::list<boost::shared_ptr<OpenSoT::Task<Matrix, Vector>>> task_list;
    task_list.push_back(min_effort_task);

    boost::shared_ptr<OpenSoT::tasks::Aggregated> joint_space_task(
                new OpenSoT::tasks::Aggregated(task_list, q.size()));


    boost::shared_ptr<OpenSoT::constraints::velocity::VelocityLimits> joint_vel_limits(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q.size()));

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> bounds_list;
    bounds_list.push_back(joint_vel_limits);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));


    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(joint_space_task);
    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    yarp::sig::Vector dq(q.size(), 0.0);
    old_gravity_gradient oldGravityGradient;
    for(unsigned int i = 0; i < 100; ++i)
    {
        joint_space_task->update(q);
        bounds->update(q);

        yarp::sig::Vector old_gradient = oldGravityGradient.computeMinEffort(q);

        for(unsigned int i = 0; i < q.size(); ++i)
            EXPECT_NEAR(joint_space_task->getb()[i], old_gradient[i], 1E-3);


        EXPECT_TRUE(sot.solve(dq));
        q += dq;
    }

}

TEST_F(testQPOases_sot, testAggregated2Tasks)
{
    iDynUtils idynutils;
    iDynUtils idynutils_com;
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);
    idynutils_com.updateiDyn3Model(q, true);
    idynutils_com.iDyn3_model.setFloatingBaseLink(idynutils_com.left_leg.index);
    idynutils.updateiDyn3Model(q, true);
    idynutils_com.updateiDyn3Model(q, true);


    // BOUNDS
        boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointLimits = OpenSoT::constraints::velocity::JointLimits::ConstraintPtr(
                                new OpenSoT::constraints::velocity::JointLimits(
                                    q,
                                    idynutils.iDyn3_model.getJointBoundMax(),
                                    idynutils.iDyn3_model.getJointBoundMin()));

        boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointVelocity = OpenSoT::constraints::velocity::VelocityLimits::ConstraintPtr(
                                new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.01,q.size()));

        boost::shared_ptr<OpenSoT::constraints::Aggregated > bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(
                    new OpenSoT::constraints::Aggregated(boundsJointLimits, boundsJointVelocity, q.size()));

    //3 Taks in Aggregated: A1 = CoM + l_arm + r_arm
    boost::shared_ptr<OpenSoT::tasks::velocity::CoM> com_task(
                new OpenSoT::tasks::velocity::CoM(q, idynutils_com));
    com_task->setLambda(1.0);
    boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsCoMVelocity = OpenSoT::constraints::velocity::CoMVelocity::ConstraintPtr(
        new OpenSoT::constraints::velocity::CoMVelocity(
                    yarp::sig::Vector(3, 0.03), 0.01 , q, idynutils_com));
    com_task->getConstraints().push_back(boundsCoMVelocity);

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

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(cartesian_task);
    cartesianTasks.push_back(cartesian_task2);
    cartesianTasks.push_back(com_task);

    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(cartesianTasks,q.size()));

    // Postural Task
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q);
    postural_task->getConstraints().push_back(boundsCoMVelocity);

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> jointTasks;
    jointTasks.push_back(postural_task);

    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > taskJointAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(jointTasks,q.size()));

    std::vector<boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(taskJointAggregated);

    //std::cout<<"J1 = ["<<taskCartesianAggregated->getA().toString()<<"]"<<std::endl;
    //std::cout<<"Aineq1 = ["<<(*(taskCartesianAggregated->getConstraints().begin()))->getAineq().toString()<<"]"<<std::endl;

    boost::shared_ptr<OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector> > sot = OpenSoT::solvers::QPOases_sot::SolverPtr(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds, 2E2));


    //SET SOME REFERENCES
    yarp::sig::Matrix T_arm_init = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex(ee1));
    yarp::sig::Matrix T_arm_ref = T_arm_init;
    KDL::Frame T_arm_ref_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm_ref, T_arm_ref_kdl);

    yarp::sig::Matrix T_arm2_init = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex(ee2));
    yarp::sig::Matrix T_arm2_ref = T_arm2_init;
    T_arm2_ref(0,3) += 0.05;
    KDL::Frame T_arm2_ref_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2_ref, T_arm2_ref_kdl);


    yarp::sig::Vector T_com_p_init = idynutils.iDyn3_model.getCOM();
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


    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        idynutils.updateiDyn3Model(q, true);
        idynutils_com.updateiDyn3Model(q, true);

        taskCartesianAggregated->update(q);
        taskJointAggregated->update(q);
        bounds->update(q);

        sot->solve(dq);
        q += dq;
    }

    yarp::sig::Matrix T_arm = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex(ee1));
    KDL::Frame T_arm_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm, T_arm_kdl);

    yarp::sig::Matrix T_arm2 = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex(ee2));
    KDL::Frame T_arm2_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_arm2, T_arm2_kdl);

    yarp::sig::Vector T_com_p = idynutils_com.iDyn3_model.getCOM();
    yarp::sig::Matrix T_com(4,4); T_com.eye();
    T_com(0,3) = T_com_p[0];
    T_com(1,3) = T_com_p[1];
    T_com(2,3) = T_com_p[2];
    KDL::Frame T_com_kdl;
    cartesian_utils::fromYARPMatrixtoKDLFrame(T_com, T_com_kdl);

    yarp::sig::Matrix T_com_init(4,4);
    T_com_init(0,3) = T_com_p_init[0];
    T_com_init(1,3) = T_com_p_init[1];
    T_com_init(2,3) = T_com_p_init[2];
    std::cout<<GREEN<<"CoM Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_com_init);
    std::cout<<GREEN<<"CoM Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_com_ref_kdl);
    std::cout<<GREEN<<"CoM Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_com_kdl);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_com_kdl.p[i], T_com_ref_kdl.p[i], 1E-3)<<"For i: "<<i<<std::endl;




    std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_arm_init);
    std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm_ref_kdl);
    std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm_kdl);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_arm_kdl.p[i], T_arm_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_arm_kdl.M(i,j), T_arm_ref_kdl.M(i,j), 1E-2);



    std::cout<<GREEN<<"Arm2 Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_arm2_init);
    std::cout<<GREEN<<"Arm2 Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm2_ref_kdl);
    std::cout<<GREEN<<"Arm2 Pose: "<<DEFAULT<<std::endl; cartesian_utils::printKDLFrame(T_arm2_kdl);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_arm2_kdl.p[i], T_arm2_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_arm2_kdl.M(i,j), T_arm2_ref_kdl.M(i,j), 1E-2);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
