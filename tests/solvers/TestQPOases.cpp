#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <XBotInterface/ModelInterface.h>


std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class old_gravity_gradient
{
public:
    XBot::ModelInterface::Ptr _model_ptr;

    old_gravity_gradient()
    {
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    }

    Eigen::VectorXd computeMinEffort(const Eigen::VectorXd& q)
    {
        Eigen::MatrixXd W(_model_ptr->getJointNum(), _model_ptr->getJointNum());
        W.setIdentity(W.rows(), W.cols());

        Eigen::VectorXd tau_max;
        _model_ptr->getEffortLimits(tau_max);

        for(unsigned int i = 0; i < _model_ptr->getJointNum(); ++i)
            W(i,i) = 1.0 / (tau_max[i]*tau_max[i]);

        return -1.0 * getGravityCompensationGradient(W, q);
    }

    Eigen::VectorXd getGravityCompensationGradient(const Eigen::MatrixXd& W, const Eigen::VectorXd& q)
    {

        /// cost function is tau_g^t*tau_g
        Eigen::VectorXd gradient(q.size()); gradient.setZero(q.size());
        Eigen::VectorXd deltas(q.size()); deltas.setZero(q.size());
        const double h = 1E-3;
        for(unsigned int i = 0; i < gradient.size(); ++i)
        {
            // forward method gradient computation, milligrad
            deltas[i] = h;
            Eigen::VectorXd tau_gravity_q_a = getGravityCompensationTorque(q+deltas);
            Eigen::VectorXd tau_gravity_q_b = getGravityCompensationTorque(q-deltas);

            double C_g_q_a = tau_gravity_q_a.dot(W*tau_gravity_q_a);
            double C_g_q_b = tau_gravity_q_b.dot(W*tau_gravity_q_b);
            gradient[i] = (C_g_q_a - C_g_q_b)/(2.0*h);
            deltas[i] = 0.0;
        }

        return gradient;
    }

    Eigen::VectorXd getGravityCompensationTorque(const Eigen::VectorXd q)
    {
        static Eigen::VectorXd zeroes(q.size()); zeroes.setZero(q.size());
        static Eigen::VectorXd tau(q.size()); tau.setZero(q.size());

        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(zeroes);
        _model_ptr->setJointAcceleration(zeroes);
        _model_ptr->update();

        _model_ptr->computeGravityCompensation(tau);

        return tau;
    }

};

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

class testiHQP: public ::testing::Test
{
protected:
    std::ofstream _log;

    testiHQP()
    {
        _log.open("testiHQP.m");
    }

    virtual ~testiHQP() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q(_model_ptr->getJointNum());
    _q.setZero(_q.size());
    _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testQPOasesProblem, test_update_constraint)
{
    OpenSoT::solvers::QPOasesBackEnd qp(3,0);
    Eigen::MatrixXd H(1,3);
    H<<1,1,1;
    Eigen::VectorXd b(1);
    b<<10;
    Eigen::MatrixXd A(0,0);
    Eigen::VectorXd lA;
    Eigen::VectorXd uA;
    Eigen::VectorXd l(3);
    l<<-10,
       -10,
       -10;
    Eigen::VectorXd u(3);
    u<<10,
       10,
       10;
    EXPECT_TRUE(qp.initProblem(H.transpose()*H,-1.*H.transpose()*b,A,lA,uA,l,u));

    EXPECT_TRUE(qp.solve());
    Eigen::VectorXd solution = qp.getSolution();
    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;

    EXPECT_NEAR(solution[0], 3.333,1E-3);
    EXPECT_NEAR(solution[1], 3.333,1E-3);
    EXPECT_NEAR(solution[2], 3.333,1E-3);

    A.resize(1,3);
    A<<1,0,1;
    lA.resize(1);
    lA<<20;
    uA.resize(1);
    uA=lA;
    EXPECT_TRUE(qp.updateConstraints(A, lA,uA));
    EXPECT_TRUE(qp.solve());
    solution = qp.getSolution();
    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;
    EXPECT_NEAR(solution[0], 10.,1E-6);
    EXPECT_NEAR(solution[1],-10.,1E-6);
    EXPECT_NEAR(solution[2], 10.,1E-6);

//    H.resize(4,3);
//    H<<1,1,1,
//       0,1,1,
//       1,1,0,
//       1,0,1;
//    b.resize(4);
//    b<<6,
//       5,
//       3,
//       3;
//    EXPECT_TRUE(qp.updateTask(H.transpose()*H, -1.*H.transpose()*b));
//    EXPECT_TRUE(qp.solve());
//    solution = qp.getSolution();
//    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;
//    EXPECT_NEAR(solution[0], .5714,1E-4);
//    EXPECT_NEAR(solution[1], 2.5714,1E-4);
//    EXPECT_NEAR(solution[2], 2.5714,1E-4);
}

TEST_F(testQPOasesProblem, test_update_task)
{
    OpenSoT::solvers::QPOasesBackEnd qp(3,0);
    Eigen::MatrixXd H(2,3);
    H<<1,1,1,
       0,1,1;
    Eigen::VectorXd b(2);
    b<<6,
       5;
    Eigen::MatrixXd A(0,0);
    Eigen::VectorXd lA;
    Eigen::VectorXd uA;
    Eigen::VectorXd l(3);
    l<<-10,
       -10,
       -10;
    Eigen::VectorXd u(3);
    u<<10,
       10,
       10;
    EXPECT_TRUE(qp.initProblem(H.transpose()*H,-1.*H.transpose()*b,A,lA,uA,l,u));

    EXPECT_TRUE(qp.solve());
    Eigen::VectorXd solution = qp.getSolution();
    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;

    EXPECT_NEAR(solution[0], 1.,1E-6);
    EXPECT_NEAR(solution[1], 2.5,1E-6);
    EXPECT_NEAR(solution[2], 2.5,1E-6);

    H.resize(3,3);
    H<<1,1,1,
       0,1,1,
       1,1,0;
    b.resize(3);
    b<<6,
       5,
       3;
    EXPECT_TRUE(qp.updateTask(H.transpose()*H, -1.*H.transpose()*b));
    EXPECT_TRUE(qp.solve());
    solution = qp.getSolution();
    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;
    EXPECT_NEAR(solution[0], 1.,1E-6);
    EXPECT_NEAR(solution[1], 2.,1E-6);
    EXPECT_NEAR(solution[2], 3.,1E-6);

    H.resize(4,3);
    H<<1,1,1,
       0,1,1,
       1,1,0,
       1,0,1;
    b.resize(4);
    b<<6,
       5,
       3,
       3;
    EXPECT_TRUE(qp.updateTask(H.transpose()*H, -1.*H.transpose()*b));
    EXPECT_TRUE(qp.solve());
    solution = qp.getSolution();
    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;
    EXPECT_NEAR(solution[0], .5714,1E-4);
    EXPECT_NEAR(solution[1], 2.5714,1E-4);
    EXPECT_NEAR(solution[2], 2.5714,1E-4);
}


/**
 * @brief TEST_F testSimpleProblem test solution of a simple CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testSimpleProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::QPOasesBackEnd testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);

    testProblem.initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u);

    EXPECT_TRUE(testProblem.solve());
    Eigen::VectorXd s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblem.solve());

        Eigen::VectorXd s = testProblem.getSolution();
        EXPECT_EQ(-sp.g[0], s[0]);
        EXPECT_EQ(-sp.g[1], s[1]);
    }

}

/**
 * @brief TEST_F testUpdatedProblem test solution of a simple NON-CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testUpdatedProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::QPOasesBackEnd testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);

    testProblem.initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u);

    EXPECT_TRUE(testProblem.solve());
    Eigen::VectorXd s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);

    sp.g[0] = -1.0; sp.g[1] = 1.0;
    testProblem.updateTask(sp.H,
                           sp.g);
    EXPECT_TRUE(testProblem.solve());

    s = testProblem.getSolution();
    EXPECT_EQ(-sp.g[0], s[0]);
    EXPECT_EQ(-sp.g[1], s[1]);
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

TEST_F(testQPOasesProblem, testTask)
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

    qpOASES::SQProblem testProblem(q.size(), 0, qpOASES::HST_IDENTITY);
    int nWSR = 132;
    qpOASES::returnValue val = testProblem.init(H.data(), g.data(), NULL, NULL, NULL, NULL,
                                                NULL, nWSR);

    EXPECT_TRUE(val == qpOASES::SUCCESSFUL_RETURN);

    Eigen::VectorXd dq(q.size());
    testProblem.getPrimalSolution(dq.data());
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i] + dq[i], q_ref[i], 1E-12);
}

TEST_F(testQPOasesTask, testQPOasesTask)
{
    Eigen::MatrixXd A;
    EXPECT_TRUE(A.data() == NULL);

    Eigen::VectorXd q_ref(10); q_ref.setZero(10);
    Eigen::VectorXd q(q_ref.size()); q.setZero(10);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();
    std::cout<<"q: "<<q<<std::endl;

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    postural_task->update(q);
    std::cout<<"error: "<<postural_task->getb()<<std::endl;

    OpenSoT::solvers::QPOasesBackEnd qp_postural_problem(postural_task->getXSize(), 0,
                                                         postural_task->getHessianAtype());

    EXPECT_TRUE(qp_postural_problem.initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                                                Eigen::VectorXd(), Eigen::VectorXd()));
    std::cout<<"solution: "<<qp_postural_problem.getSolution()<<std::endl;
    Eigen::VectorXd dq = qp_postural_problem.getSolution();
    q += dq;

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(q[i], q_ref[i]);

}

using namespace OpenSoT::constraints::velocity;
TEST_F(testQPOasesTask, testProblemWithConstraint)
{
        XBot::ModelInterface::Ptr _model_ptr;
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
        Eigen::VectorXd q_ref(q.size()); q_ref.setConstant(q.size(), M_PI);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
        postural_task->setReference(q_ref);

        Eigen::VectorXd q_max, q_min;
        _model_ptr->getJointLimits(q_min, q_max);


        JointLimits::Ptr joint_limits(
            new JointLimits(q, q_max, q_min));
        postural_task->getConstraints().push_back(joint_limits);
        postural_task->setLambda(0.1);

        OpenSoT::solvers::QPOasesBackEnd qp_postural_problem(postural_task->getXSize(), 0,
                                                             postural_task->getHessianAtype());
        std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraint_list =
                postural_task->getConstraints();
        OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr constraint = constraint_list.front();
        EXPECT_TRUE(qp_postural_problem.initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                    Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                                                    constraint->getLowerBound(), constraint->getUpperBound()));

        Eigen::VectorXd l_old = qp_postural_problem.getl();
        Eigen::VectorXd u_old = qp_postural_problem.getu();

        EXPECT_TRUE(l_old == q_min);
        EXPECT_TRUE(u_old == q_max);

        Eigen::VectorXd l, u;
        for(unsigned int i = 0; i < 100; ++i)
        {
            postural_task->update(q);

            qp_postural_problem.updateBounds(constraint->getLowerBound(), constraint->getUpperBound());
            qp_postural_problem.updateTask(postural_task->getA(), -1.0*postural_task->getb());

            EXPECT_TRUE(qp_postural_problem.solve());
            l = qp_postural_problem.getl();
            u = qp_postural_problem.getu();
            Eigen::VectorXd dq = qp_postural_problem.getSolution();
            q += dq;

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 0; i < q.size(); ++i)
        {
            if(q_ref[i] >= q_max[i])
            {
                std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], q_max(i), 1E-4);
            }
            else if(q_ref[i] <= q_min[i])
            {
                std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], q_min[i], 1E-4);
            }
            else
                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
        }
}

TEST_F(testQPOasesTask, test_on_eigen)
{
    Eigen::MatrixXd A(2,2);
    A<<1,2,3,4;

    std::cout<<"A: "<<A<<std::endl;

    Eigen::MatrixXd B(1,2);
    B<<5,6;

    std::cout<<"B: "<<B<<std::endl;

    Eigen::MatrixXd C(A.rows()+B.rows(), A.cols());
    C<<A,B;

    std::cout<<"C: "<<C<<std::endl;

    A.conservativeResize(A.rows()+B.rows(), A.cols());
    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;

    std::cout<<"A: "<<A<<std::endl;

    Eigen::VectorXd a(3);
    a<<1,2,3;

    Eigen::VectorXd b(2);
    b<<4,5;

    a.conservativeResize(a.rows()+b.rows());
    a.segment(a.rows()-b.rows(),b.rows())<<b;

    std::cout<<"a: "<<a<<std::endl;
}

TEST_F(testiHQP, testContructor1Problem)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
    Eigen::VectorXd q_ref(q.size()); q_ref.setConstant(q.size(), M_PI);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);


    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits(q_min, q_max);

    JointLimits::Ptr joint_limits(
        new JointLimits(q,
                        q_max,
                        q_min));
    postural_task->setLambda(0.1);

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list;
    bounds_list.push_back(joint_limits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(q.size());
    dq.setZero(q.size());
    for(unsigned int i = 0; i < 100; ++i)
    {
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot.solve(dq));
        Eigen::VectorXd _dq = dq;
        q += _dq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
    {
        if(q_ref[i] >= q_max[i])
        {
            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], q_max[i], 1E-4);
        }
        else if(q_ref[i] <= q_min[i])
        {
            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], q_min[i], 1E-4);
        }
        else
            EXPECT_NEAR( q[i], q_ref[i], 1E-4);

    }
}

TEST_F(testQPOasesTask, testCoMTask)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::CoM::Ptr com_task(
                new OpenSoT::tasks::velocity::CoM(q, *_model_ptr));

    Eigen::VectorXd tmp(3);
    tmp<<0.06,0.06,0.06;
    OpenSoT::constraints::velocity::CoMVelocity::Ptr com_vel_constr(
                new OpenSoT::constraints::velocity::CoMVelocity(
                    tmp,1.0,q,*_model_ptr));
    com_task->getConstraints().push_back(com_vel_constr);

    std::list< OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraint_list =
            com_task->getConstraints();
    OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr constraint = constraint_list.front();

    OpenSoT::solvers::QPOasesBackEnd qp_CoM_problem(com_task->getXSize(), constraint->getAineq().rows(),
                                                    com_task->getHessianAtype());
    ASSERT_TRUE(qp_CoM_problem.initProblem(com_task->getA().transpose()*com_task->getA(), -1.0*com_task->getA().transpose()*com_task->getb(),
                                                constraint->getAineq(), constraint->getbLowerBound(), constraint->getbUpperBound(),
                                                Eigen::VectorXd(), Eigen::VectorXd()));


    Eigen::VectorXd com_i = com_task->getActualPosition();
    Eigen::VectorXd com_f = com_i;
    com_f[0] += 0.05;
    com_f[1] += 0.05;
    com_f[2] -= 0.05;
    com_task->setReference(com_f);


    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        com_task->update(q);

        qp_CoM_problem.updateProblem(com_task->getA().transpose()*com_task->getA(), -1.0*com_task->getA().transpose()*com_task->getb(),
                                          constraint->getAineq(), constraint->getbLowerBound(), constraint->getbUpperBound(),
                                          Eigen::VectorXd(), Eigen::VectorXd());

        ASSERT_TRUE(qp_CoM_problem.solve());
        Eigen::VectorXd dq = qp_CoM_problem.getSolution();
        q += dq;
    }


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR( com_task->getb()[i], 0.0, 1E-4);

}

TEST_F(testQPOasesTask, testCartesian)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::Affine3d T;
    _model_ptr->getPose("l_wrist", "Waist", T);


    //2 Tasks: Cartesian & Postural
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, *_model_ptr,
                "l_wrist", "Waist"));

    cartesian_task->update(q);

    Eigen::MatrixXd T_actual = cartesian_task->getActualPose();
    std::cout<<"T_actual: \n"<<T_actual<<std::endl;


    Eigen::MatrixXd T_ref = T.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.02;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

    OpenSoT::solvers::QPOasesBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype());
    ASSERT_TRUE(qp_cartesian_problem.initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.003*Eigen::VectorXd::Ones(q.size()), 0.003*Eigen::VectorXd::Ones(q.size())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(q);
        qp_cartesian_problem.updateTask(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem.solve());
        Eigen::VectorXd dq = qp_cartesian_problem.getSolution();
        q += dq;
    }

    _model_ptr->getPose("l_wrist", "Waist", T);

    std::cout<<"T: \n"<<T.matrix()<<std::endl;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T(i,3), T_ref(i,3), 1E-4);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-4);
}

TEST_F(testiHQP, testContructor2Problems)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    Eigen::VectorXd torso(q.size()); torso.setZero(q.size());
    torso[0] = getRandomAngle(-20.0*M_PI/180.0, 20.0*M_PI/180.0);
    torso[1] = getRandomAngle(0.0, 45.0*M_PI/180.0);
    torso[2] = getRandomAngle(-30.0*M_PI/180.0, 30.0*M_PI/180.0);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::Affine3d T_init;
    _model_ptr->getPose("l_wrist", "Waist", T_init);
    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;


    //2 Tasks: Cartesian & Postural
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::l_wrist", q, *_model_ptr,
                                                       "l_wrist", "Waist"));
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(q));

    postural_task->setReference(q);
    cartesian_task->setReference(T_init.matrix());

    int t = 50;
    //Constraints set to the Cartesian Task
    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits(q_min, q_max);

    JointLimits::Ptr joint_limits(
        new JointLimits(q, q_max,
                           q_min));
    joint_limits->setBoundScaling((double)(1.0/t));

    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(0.3, (double)(1.0/t), q.size()));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    //Create the SoT
    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    std::cout<<"Initial Position Error: "<<cartesian_task->positionError<<std::endl;
    std::cout<<"Initial Orientation Error: "<<cartesian_task->orientationError<<std::endl;

    OpenSoT::solvers::iHQP sot(stack_of_tasks, joint_constraints);


    KDL::Frame T_ref_kdl;
    T_ref_kdl.p[0] = 0.283; T_ref_kdl.p[1] = 0.156; T_ref_kdl.p[2] = 0.499;
    T_ref_kdl.M = T_ref_kdl.M.Quaternion(0.0, 0.975, 0.0, -0.221);
    cartesian_task->setReference(T_ref_kdl);

    //Solve SoT
    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    Eigen::VectorXd dq(q.size());
    dq.setZero(q.size());
    for(unsigned int i = 0; i < 10*t; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        cartesian_task->update(q);
        postural_task->update(q);
        joint_constraints->update(q);

        ASSERT_TRUE(sot.solve(dq));
        Eigen::VectorXd _dq = dq;
        //std::cout<<"Solution: ["<<dq<<"]"<<std::endl;
        q += _dq;
    }

    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;
    KDL::Frame T_kdl;
    _model_ptr->getPose("l_wrist", "Waist", T_kdl);
    std::cout<<"FINAL CONFIG: "<<T_kdl<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref_kdl<<std::endl;


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.p[i], T_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.M(i,j), T_ref_kdl.M(i,j), 1E-2);


}

TEST_F(testiHQP, testContructor1ProblemAggregated)
{
    int n_dofs = 5;
    Eigen::VectorXd q(n_dofs); q.setZero(n_dofs);
    Eigen::VectorXd q_ref(n_dofs); q_ref.setConstant(n_dofs, M_PI);

    Eigen::VectorXd q2(n_dofs); q2.setZero(n_dofs);
    Eigen::VectorXd q_ref2(n_dofs); q_ref2.setConstant(n_dofs, M_PI);


    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural_task2(
            new OpenSoT::tasks::velocity::Postural(q2));
    postural_task2->setReference(q_ref2);
    std::list<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> task_list;
    task_list.push_back(postural_task2);
    OpenSoT::tasks::Aggregated::Ptr joint_space_task(
                new OpenSoT::tasks::Aggregated(task_list, q2.size()));


    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_vel_limits(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q.size()));
    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_vel_limits2(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q2.size()));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list;
    bounds_list.push_back(joint_vel_limits);
    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list2;
    bounds_list2.push_back(joint_vel_limits2);


    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));
    OpenSoT::constraints::Aggregated::Ptr bounds2(
                new OpenSoT::constraints::Aggregated(bounds_list2, q2.size()));

//1. Here we use postural_task
    std::vector<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> stack_of_tasks;
    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(q.size());
    dq.setZero(q.size());
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
    std::vector<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> stack_of_tasks2;
    stack_of_tasks2.push_back(joint_space_task);
    OpenSoT::solvers::iHQP sot2(stack_of_tasks2, bounds2);

    EXPECT_TRUE(sot2.getNumberOfTasks() == 1);
    Eigen::VectorXd dq2(q2.size());
    dq2.setZero(q2.size());
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

TEST_F(testiHQP, testMinEffort)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
    q[_model_ptr->getDofIndex("LAnkSag")] = 45.0 * M_PI/180.0;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::MinimumEffort::Ptr min_effort_task(
            new OpenSoT::tasks::velocity::MinimumEffort(
                    q, *_model_ptr));

    std::list<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> task_list;
    task_list.push_back(min_effort_task);

    OpenSoT::tasks::Aggregated::Ptr joint_space_task(
                new OpenSoT::tasks::Aggregated(task_list, q.size()));


    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_vel_limits(
        new OpenSoT::constraints::velocity::VelocityLimits(0.3, 0.1, q.size()));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list;
    bounds_list.push_back(joint_vel_limits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(joint_space_task);
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(q.size());
    dq.setZero(q.size());
    old_gravity_gradient oldGravityGradient;
    for(unsigned int i = 0; i < 100; ++i)
    {
        joint_space_task->update(q);
        bounds->update(q);

        Eigen::VectorXd old_gradient = oldGravityGradient.computeMinEffort(q);

        for(unsigned int i = 0; i < q.size(); ++i)
            EXPECT_NEAR(joint_space_task->getb()[i], old_gradient[i], 1E-3);


        EXPECT_TRUE(sot.solve(dq));
        Eigen::VectorXd dq_ = dq;
        q += dq_;
    }

}

TEST_F(testQPOasesProblem, testNullHessian)
{
    OpenSoT::solvers::QPOasesBackEnd qp(30, 0, OpenSoT::HessianType::HST_ZERO, 1e10);

    Eigen::MatrixXd H(30,30); H.setZero(30,30);
    Eigen::VectorXd g(30); g.setRandom(30);

    ASSERT_TRUE(qp.initProblem(H, g,
                   Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                   Eigen::VectorXd(), Eigen::VectorXd()));

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
