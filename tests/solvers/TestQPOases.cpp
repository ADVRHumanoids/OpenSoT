#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include <eigen_conversions/eigen_kdl.h>

#include "../common.h"

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class old_gravity_gradient
{
public:
    XBot::ModelInterface::Ptr _model_ptr;

    old_gravity_gradient()
    {
        _model_ptr = GetTestModel("coman_floating_base");
    }

    Eigen::VectorXd computeMinEffort(const Eigen::VectorXd& q)
    {
        Eigen::MatrixXd W(_model_ptr->getNv(), _model_ptr->getNv());
        W.setIdentity(W.rows(), W.cols());

        Eigen::VectorXd tau_max;
        _model_ptr->getEffortLimits(tau_max);

        for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
            W(i,i) = 1.0 / (tau_max[i]*tau_max[i]);

        return -1.0 * getGravityCompensationGradient(W, q);
    }

    Eigen::VectorXd getGravityCompensationGradient(const Eigen::MatrixXd& W, const Eigen::VectorXd& q)
    {

        /// cost function is tau_g^t*tau_g
        Eigen::VectorXd gradient(_model_ptr->getNv()); gradient.setZero();
        Eigen::VectorXd deltas(_model_ptr->getNv()); deltas.setZero();
        const double h = 1E-3;


        for(unsigned int i = 0; i < gradient.size(); ++i)
        {
            // forward method gradient computation, milligrad
            deltas[i] = h;
            Eigen::VectorXd tau_gravity_q_a = getGravityCompensationTorque(_model_ptr->sum(q, deltas));
            Eigen::VectorXd tau_gravity_q_b = getGravityCompensationTorque(_model_ptr->sum(q, -deltas));

            double C_g_q_a = tau_gravity_q_a.dot(W*tau_gravity_q_a);
            double C_g_q_b = tau_gravity_q_b.dot(W*tau_gravity_q_b);
            gradient[i] = (C_g_q_a - C_g_q_b)/(2.0*h);
            deltas[i] = 0.0;
        }

        return gradient;
    }

    Eigen::VectorXd getGravityCompensationTorque(const Eigen::VectorXd q)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        Eigen::VectorXd tau(_model_ptr->getNv());
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

class testQPOasesProblem : public TestBase
{
protected:

    testQPOasesProblem(): TestBase("coman_floating_base")
    {

    }

    virtual ~testQPOasesProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

class testQPOasesTask: public TestBase
{
protected:

    testQPOasesTask(): TestBase("coman_floating_base")
    {

    }

    virtual ~testQPOasesTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testiHQP : public TestBase
{
protected:
    std::ofstream _log;

    testiHQP(): TestBase("coman_floating_base")
    {
        _log.open("testiHqp->m");
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
    Eigen::VectorXd _q;
    _q = _model_ptr->getNeutralQ();
    _q[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")+1 ] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")+1 ] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")+1 ] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")+1 ] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")+1 ] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")+1 ] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testQPOasesProblem, test_update_constraint)
{
    //OpenSoT::solvers::QPOasesBackEnd qp(3,0);


    Eigen::MatrixXd H(1,3);
    H<<1,1,1;
    Eigen::VectorXd b(1);
    b<<10;
    Eigen::MatrixXd A(1,3); A.setZero(1,3);
    Eigen::VectorXd lA(1); lA.setZero(1);
    Eigen::VectorXd uA(1); uA.setZero(1);
    Eigen::VectorXd l(3);
    l<<-10,
       -10,
       -10;
    Eigen::VectorXd u(3);
    u<<10,
       10,
       10;

    OpenSoT::solvers::BackEnd::Ptr qp = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, 3, 1, OpenSoT::HST_SEMIDEF,1e4);
    EXPECT_TRUE(qp->initProblem(H.transpose()*H,-1.*H.transpose()*b,A,lA,uA,l,u));

    EXPECT_TRUE(qp->solve());
    Eigen::VectorXd solution = qp->getSolution();
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
    EXPECT_TRUE(qp->updateConstraints(A, lA,uA));
    EXPECT_TRUE(qp->solve());
    solution = qp->getSolution();
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
//    EXPECT_TRUE(qp->updateTask(H.transpose()*H, -1.*H.transpose()*b));
//    EXPECT_TRUE(qp->solve());
//    solution = qp->getSolution();
//    std::cout<<"solution is: ["<<solution<<"]"<<std::endl;
//    EXPECT_NEAR(solution[0], .5714,1E-4);
//    EXPECT_NEAR(solution[1], 2.5714,1E-4);
//    EXPECT_NEAR(solution[2], 2.5714,1E-4);
}

TEST_F(testQPOasesProblem, test_update_task)
{
    //OpenSoT::solvers::QPOasesBackEnd qp(3,0);
    OpenSoT::solvers::BackEnd::Ptr qp = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, 3, 0, OpenSoT::HST_UNKNOWN,1.);

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
    EXPECT_TRUE(qp->initProblem(H.transpose()*H,-1.*H.transpose()*b,A,lA,uA,l,u));

    EXPECT_TRUE(qp->solve());
    Eigen::VectorXd solution = qp->getSolution();
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
    EXPECT_TRUE(qp->updateTask(H.transpose()*H, -1.*H.transpose()*b));
    EXPECT_TRUE(qp->solve());
    solution = qp->getSolution();
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
    EXPECT_TRUE(qp->updateTask(H.transpose()*H, -1.*H.transpose()*b));
    EXPECT_TRUE(qp->solve());
    solution = qp->getSolution();
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

    //OpenSoT::solvers::QPOasesBackEnd testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);
    OpenSoT::solvers::BackEnd::Ptr testProblem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,1e-9);

    testProblem->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u);

    EXPECT_TRUE(testProblem->solve());
    Eigen::VectorXd s = testProblem->getSolution();
    EXPECT_NEAR(-sp.g[0], s[0],1e-14)<<"-sp.g[0]: "<<-sp.g[0]<<"  VS  s[0]: "<<s[0]<<std::endl;
    EXPECT_NEAR(-sp.g[1], s[1],1e-14)<<"-sp.g[1]: "<<-sp.g[1]<<"  VS  s[1]: "<<s[1]<<std::endl;

    for(unsigned int i = 0; i < 1000; ++i)
    {
        EXPECT_TRUE(testProblem->solve());

        Eigen::VectorXd s = testProblem->getSolution();
        EXPECT_NEAR(-sp.g[0], s[0],1e-14);
        EXPECT_NEAR(-sp.g[1], s[1],1e-14);
    }

}

/**
 * @brief TEST_F testUpdatedProblem test solution of a simple NON-CONSTANT QP problem
 */
TEST_F(testQPOasesProblem, testUpdatedProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    //OpenSoT::solvers::QPOasesBackEnd testProblem(x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht);
    OpenSoT::solvers::BackEnd::Ptr testProblem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,1e-9);

    testProblem->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u);

    EXPECT_TRUE(testProblem->solve());
    Eigen::VectorXd s = testProblem->getSolution();
    EXPECT_NEAR(-sp.g[0], s[0],1e-11);
    EXPECT_NEAR(-sp.g[1], s[1],1e-11);

    sp.g[0] = -1.0; sp.g[1] = 1.0;
    testProblem->updateTask(sp.H,
                           sp.g);
    EXPECT_TRUE(testProblem->solve());

    s = testProblem->getSolution();
    EXPECT_NEAR(-sp.g[0], s[0],1e-14);
    EXPECT_NEAR(-sp.g[1], s[1],1e-14);
}


TEST_F(testQPOasesProblem, testTask)
{
    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural postural_task(*_model_ptr);
    postural_task.setReference(q_ref);
    postural_task.update(Eigen::VectorXd(0));

    Eigen::MatrixXd H(_model_ptr->getNv(),_model_ptr->getNv()); H.setIdentity();
    Eigen::VectorXd g(-1.0*postural_task.getb());

    qpOASES::SQProblem testProblem(_model_ptr->getNv(), 0, qpOASES::HST_IDENTITY);

    int nWSR = 132;
    qpOASES::returnValue val = testProblem.init(H.data(), g.data(), NULL, NULL, NULL, NULL,
                                                NULL, nWSR);


    EXPECT_TRUE(val == qpOASES::SUCCESSFUL_RETURN);

    Eigen::VectorXd dq(_model_ptr->getNv());
    testProblem.getPrimalSolution(dq.data());
    q = _model_ptr->sum(q, dq);
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i], q_ref[i], 1E-12);
}

TEST_F(testQPOasesTask, testQPOasesTask)
{
    Eigen::MatrixXd A;
    EXPECT_TRUE(A.data() == NULL);

    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();
    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    std::cout<<"q: "<<q<<std::endl;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    postural_task->setReference(q_ref);
    postural_task->update(Eigen::VectorXd(0));
    std::cout<<"error: "<<postural_task->getb()<<std::endl;

//    OpenSoT::solvers::QPOasesBackEnd qp_postural_problem(postural_task->getXSize(), 0,
//                                                         postural_task->getHessianAtype());
    OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, postural_task->getXSize(), 0,
                postural_task->getHessianAtype(),1.);

    EXPECT_TRUE(qp_postural_problem->initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                                                Eigen::VectorXd(), Eigen::VectorXd()));
    std::cout<<"solution: "<<qp_postural_problem->getSolution()<<std::endl;
    Eigen::VectorXd dq = qp_postural_problem->getSolution();
    q = _model_ptr->sum(q, dq);

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1e-9);

}

using namespace OpenSoT::constraints::velocity;
TEST_F(testQPOasesTask, testProblemWithConstraint)
{
        Eigen::VectorXd q = _model_ptr->getNeutralQ();
        Eigen::VectorXd q_ref = _model_ptr->generateRandomQ();
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(*_model_ptr));
        postural_task->setReference(q_ref);

        Eigen::VectorXd q_max, q_min;
        _model_ptr->getJointLimits(q_min, q_max);


        JointLimits::Ptr joint_limits(
            new JointLimits(*_model_ptr, q_max, q_min));
        postural_task->getConstraints().push_back(joint_limits);
        postural_task->setLambda(1.);

        std::cout<<"postural_task->getXSize(): "<<postural_task->getXSize()<<std::endl;

//        OpenSoT::solvers::QPOasesBackEnd qp_postural_problem(postural_task->getXSize(), 0,
//                                                             postural_task->getHessianAtype());
        OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                    OpenSoT::solvers::solver_back_ends::qpOASES, postural_task->getXSize(), 0,
                    postural_task->getHessianAtype(),1.);

        std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraint_list =
                postural_task->getConstraints();
        OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr constraint = constraint_list.front();
        EXPECT_TRUE(qp_postural_problem->initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                    Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                                                    constraint->getLowerBound(), constraint->getUpperBound()));

        Eigen::VectorXd l_old = qp_postural_problem->getl();
        Eigen::VectorXd u_old = qp_postural_problem->getu();

        EXPECT_TRUE(l_old == q_min);
        EXPECT_TRUE(u_old == q_max);

        Eigen::VectorXd l, u;
        for(unsigned int i = 0; i < 100; ++i)
        {
            _model_ptr->setJointPosition(q);
            _model_ptr->update();

            postural_task->update(Eigen::VectorXd(0));

            qp_postural_problem->updateBounds(constraint->getLowerBound(), constraint->getUpperBound());
            qp_postural_problem->updateTask(postural_task->getA(), -1.0*postural_task->getb());

            EXPECT_TRUE(qp_postural_problem->solve());
            l = qp_postural_problem->getl();
            u = qp_postural_problem->getu();
            Eigen::VectorXd dq = qp_postural_problem->getSolution();
            q = _model_ptr->sum(q, dq);

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 7; i < q.size(); ++i)
        {
            if(q_ref[i] >= q_max[i-1])
            {
                std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], q_max(i-1), 1E-4);
            }
            else if(q_ref[i] <= q_min[i-1])
            {
                std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
                EXPECT_NEAR( q[i], q_min[i-1], 1E-4);
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
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    Eigen::VectorXd q_ref = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    postural_task->setReference(q_ref);


    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits(q_min, q_max);

    JointLimits::Ptr joint_limits(
        new JointLimits(*_model_ptr,
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
    double obj;
    sot.getObjective(0, obj);
    obj = norm(obj);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    double obj_;
    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update(Eigen::VectorXd(0));
        bounds->update(Eigen::VectorXd(0));

        EXPECT_TRUE(sot.solve(dq));
        sot.getObjective(0,obj_);
        obj_ = norm(obj_);
        EXPECT_TRUE(obj_ <= obj);
        obj = obj_;
        q = _model_ptr->sum(q, dq);

        std::cout<<"obj: "<<obj<<std::endl;
    }

    for(unsigned int i = 7; i < q.size(); ++i)
    {
        if(q_ref[i] >= q_max[i-1])
        {
            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], q_max[i-1], 1E-4);
        }
        else if(q_ref[i] <= q_min[i-1])
        {
            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q[i], q_min[i-1], 1E-4);
        }
        else
            EXPECT_NEAR( q[i], q_ref[i], 1E-4);

    }
}

TEST_F(testQPOasesTask, testCoMTask)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::CoM::Ptr com_task(
                new OpenSoT::tasks::velocity::CoM(*_model_ptr));

    Eigen::Vector3d tmp;
    tmp<<0.06,0.06,0.06;
    OpenSoT::constraints::velocity::CartesianVelocity::Ptr com_vel_constr(
                new OpenSoT::constraints::velocity::CartesianVelocity(
                    tmp,1.0,std::make_shared<OpenSoT::tasks::velocity::CoM>(*_model_ptr)));
    com_task->getConstraints().push_back(com_vel_constr);

    std::list< OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraint_list =
            com_task->getConstraints();
    OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr constraint = constraint_list.front();

//    OpenSoT::solvers::QPOasesBackEnd qp_CoM_problem(com_task->getXSize(), constraint->getAineq().rows(),
//                                                    com_task->getHessianAtype());
    OpenSoT::solvers::BackEnd::Ptr qp_CoM_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, com_task->getXSize(), constraint->getAineq().rows(),
                com_task->getHessianAtype(),1.);

    ASSERT_TRUE(qp_CoM_problem->initProblem(com_task->getA().transpose()*com_task->getA(), -1.0*com_task->getA().transpose()*com_task->getb(),
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

        com_task->update(Eigen::VectorXd(0));

        qp_CoM_problem->updateProblem(com_task->getA().transpose()*com_task->getA(), -1.0*com_task->getA().transpose()*com_task->getb(),
                                          constraint->getAineq(), constraint->getbLowerBound(), constraint->getbUpperBound(),
                                          Eigen::VectorXd(), Eigen::VectorXd());

        ASSERT_TRUE(qp_CoM_problem->solve());
        Eigen::VectorXd dq = qp_CoM_problem->getSolution();
        q = _model_ptr->sum(q, dq);
    }


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR( com_task->getb()[i], 0.0, 1E-4);

}

TEST_F(testQPOasesTask, testCartesian)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::Affine3d T;
    _model_ptr->getPose("l_wrist", "Waist", T);


    //2 Tasks: Cartesian & Postural
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", *_model_ptr,
                "l_wrist", "Waist"));

    cartesian_task->update(Eigen::VectorXd(0));

    Eigen::MatrixXd T_actual = cartesian_task->getActualPose();
    std::cout<<"T_actual: \n"<<T_actual<<std::endl;


    Eigen::MatrixXd T_ref = T.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.02;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

//    OpenSoT::solvers::QPOasesBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype());
    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype(),1.);

    ASSERT_TRUE(qp_cartesian_problem->initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.003*Eigen::VectorXd::Ones(_model_ptr->getNv()), 0.003*Eigen::VectorXd::Ones(_model_ptr->getNv())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(q);
        qp_cartesian_problem->updateTask(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem->solve());
        Eigen::VectorXd dq = qp_cartesian_problem->getSolution();
        q = _model_ptr->sum(q, dq);
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

TEST_F(testQPOasesTask, testEpsRegularisation)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::Affine3d T;
    _model_ptr->getPose("l_wrist", "Waist", T);


    //2 Tasks: Cartesian & Postural
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", *_model_ptr,
                "l_wrist", "Waist"));

    cartesian_task->update(Eigen::VectorXd(0));

    Eigen::MatrixXd T_actual = cartesian_task->getActualPose();
    std::cout<<"T_actual: \n"<<T_actual<<std::endl;


    Eigen::MatrixXd T_ref = T.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.02;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    cartesian_task->setReference(T_ref);
    cartesian_task->update(Eigen::VectorXd(0));

//    OpenSoT::solvers::QPOasesBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype());
    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, cartesian_task->getXSize(), 0, cartesian_task->getHessianAtype(),1.);


    EXPECT_EQ(qp_cartesian_problem->getEpsRegularisation(), 2.221e-13);

    qp_cartesian_problem->setEpsRegularisation(1e-4);
    EXPECT_EQ(qp_cartesian_problem->getEpsRegularisation(), 1e-4);


    ASSERT_TRUE(qp_cartesian_problem->initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.003*Eigen::VectorXd::Ones(_model_ptr->getNv()), 0.003*Eigen::VectorXd::Ones(_model_ptr->getNv())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(q);
        qp_cartesian_problem->updateTask(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem->solve());
        Eigen::VectorXd dq = qp_cartesian_problem->getSolution();
        q = _model_ptr->sum(q, dq);
    }

    _model_ptr->getPose("l_wrist", "Waist", T);

    std::cout<<"T: \n"<<T.matrix()<<std::endl;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T(i,3), T_ref(i,3), 1E-4);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-4);

    qp_cartesian_problem->printProblemInformation(0, "problem_id", "constraints_id", "bounds_id");
}

TEST_F(testiHQP, testContructor2Problems)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::Affine3d T_init;
    _model_ptr->getPose("l_wrist", "Waist", T_init);
    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;


    //2 Tasks: Cartesian & Postural
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::l_wrist", *_model_ptr,
                                                       "l_wrist", "Waist"));
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(*_model_ptr));

    postural_task->setReference(q);
    cartesian_task->setReference(T_init.matrix());

    int t = 50;
    //Constraints set to the Cartesian Task
    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits(q_min, q_max);

    JointLimits::Ptr joint_limits(
        new JointLimits(*_model_ptr, q_max, q_min));
    joint_limits->setBoundScaling((double)(1.0/t));

    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(*_model_ptr, 0.3, (double)(1.0/t)));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, _model_ptr->getNv()));

    //Create the SoT
    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    std::cout<<"Initial Position Error: "<<cartesian_task->getError().head(3)<<std::endl;
    std::cout<<"Initial Orientation Error: "<<cartesian_task->getError().tail(3)<<std::endl;

    OpenSoT::solvers::iHQP sot(stack_of_tasks, joint_constraints);


    KDL::Frame T_ref_kdl;
    T_ref_kdl.p[0] = 0.283; T_ref_kdl.p[1] = 0.156; T_ref_kdl.p[2] = 0.499;
    T_ref_kdl.M = T_ref_kdl.M.Quaternion(0.0, 0.975, 0.0, -0.221);
    cartesian_task->setReference(T_ref_kdl);

    //Solve SoT
    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    for(unsigned int i = 0; i < 10*t; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        cartesian_task->update(Eigen::VectorXd(0));
        postural_task->update(Eigen::VectorXd(0));
        joint_constraints->update(Eigen::VectorXd(0));

        ASSERT_TRUE(sot.solve(dq));
        q = _model_ptr->sum(q, dq);
    }

    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;
    Eigen::Affine3d T = _model_ptr->getPose("l_wrist", "Waist");
    KDL::Frame T_kdl;
    tf::transformEigenToKDL(T, T_kdl);
    // std::cout<<"FINAL CONFIG: "<< T_kdl<<std::endl;
    // std::cout<<"DESIRED CONFIG: "<<T_ref_kdl<<std::endl;




    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.p[i], T_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.M(i,j), T_ref_kdl.M(i,j), 1E-2);

    OpenSoT::solvers::BackEnd::Ptr back_end_i;
    EXPECT_FALSE(sot.getBackEnd(2, back_end_i));

    EXPECT_TRUE(sot.getBackEnd(1, back_end_i));
    EXPECT_TRUE(back_end_i->getH() ==
                postural_task->getA().transpose()*postural_task->getA()+back_end_i->getEpsRegularisation()*
                Eigen::MatrixXd::Identity(postural_task->getA().rows(),postural_task->getA().cols())); //<--because we added manual regularisation in qpoases
}


TEST_F(testiHQP, testSingleTask)
{
    Eigen::VectorXd q0 = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q0);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural;
    postural.reset(new OpenSoT::tasks::velocity::Postural(*_model_ptr));

    Eigen::VectorXd qmin(_model_ptr->getNv()), qmax(_model_ptr->getNv());
    qmax = 1000. * Eigen::VectorXd::Ones(_model_ptr->getNv());
    qmin = -qmax;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits;
    joint_limits.reset(new OpenSoT::constraints::velocity::JointLimits(*_model_ptr, qmax, qmin));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_limits;
    vel_limits.reset(new OpenSoT::constraints::velocity::VelocityLimits(*_model_ptr, 2., 0.001));

    OpenSoT::AutoStack::Ptr autostack(new OpenSoT::AutoStack(postural));
    autostack = autostack<<joint_limits<<vel_limits;
    autostack->update(Eigen::VectorXd(0));

    OpenSoT::solvers::iHQP::Ptr solver;
    solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e8);

    Eigen::VectorXd x(_model_ptr->getNv());
    EXPECT_TRUE(solver->solve(x));

    autostack->update(Eigen::VectorXd(0));

    solver.reset();
    solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e8);
    EXPECT_TRUE(solver->solve(x));
}

TEST_F(testiHQP, testMinEffort)
{

    Eigen::VectorXd q(_model_ptr->getJointNum()); q = _model_ptr->getNeutralQ();
    q[_model_ptr->getDofIndex("LAnkSag")] = 45.0 * M_PI/180.0;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::MinimumEffort::Ptr min_effort_task(
            new OpenSoT::tasks::velocity::MinimumEffort(*_model_ptr));

    std::list<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> task_list;
    task_list.push_back(min_effort_task);

    OpenSoT::tasks::Aggregated::Ptr joint_space_task(
                new OpenSoT::tasks::Aggregated(task_list, _model_ptr->getNv()));


    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_vel_limits(
        new OpenSoT::constraints::velocity::VelocityLimits(*_model_ptr, 0.3, 0.1));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list;
    bounds_list.push_back(joint_vel_limits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, _model_ptr->getNv()));


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(joint_space_task);
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds);

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    old_gravity_gradient oldGravityGradient;
    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        joint_space_task->update(Eigen::VectorXd(0));
        bounds->update(Eigen::VectorXd(0));

        Eigen::VectorXd old_gradient = oldGravityGradient.computeMinEffort(q);

        for(unsigned int i = 0; i < joint_space_task->getb().size(); ++i)
            EXPECT_NEAR(joint_space_task->getb()[i], old_gradient[i], 1E-3);


        EXPECT_TRUE(sot.solve(dq));
        q = _model_ptr->sum(q, dq);
    }

}

TEST_F(testQPOasesProblem, testNullHessian)
{
    //OpenSoT::solvers::QPOasesBackEnd qp(30, 0, OpenSoT::HessianType::HST_ZERO, 1e10);
    OpenSoT::solvers::BackEnd::Ptr qp = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, 3, 0, OpenSoT::HST_ZERO,1e10);

    Eigen::MatrixXd H(30,30); H.setZero(30,30);
    Eigen::VectorXd g(30); g.setRandom(30);

    ASSERT_TRUE(qp->initProblem(H, g,
                   Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                   Eigen::VectorXd(), Eigen::VectorXd()));

}

TEST_F(testQPOasesProblem, testResetSolverPrint)
{
    OpenSoT::solvers::QPOasesBackEnd::Ptr qp;
    std::cout<<"-------------FIRST RESET----------------"<<std::endl;
    qp.reset();
    qp = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, 30, 0,
                OpenSoT::HessianType::HST_IDENTITY,1e10);
    std::cout<<"qp ADDRESS: "<<qp<<std::endl;

    Eigen::MatrixXd H(30,30); H.setIdentity(30,30);
    Eigen::VectorXd g(30); g.setRandom(30);

    ASSERT_TRUE(qp->initProblem(H, g,
                   Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                   Eigen::VectorXd(), Eigen::VectorXd()));
    std::cout<<"-------------START FIRST LOOP----------------"<<std::endl;
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_TRUE(qp->solve());

    qp.reset();
    std::cout<<"-------------SECOND RESET----------------"<<std::endl;
    auto tmp = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, 30, 0,
                OpenSoT::HessianType::HST_IDENTITY,1e10);
    std::cout<<"tmp ADDRESS: "<<tmp<<std::endl;
    qp = tmp;
    std::cout<<"qp ADDRESS: "<<qp<<std::endl;
    ASSERT_TRUE(qp->initProblem(H, g,
                   Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                   Eigen::VectorXd(), Eigen::VectorXd()));
    std::cout<<"-------------START SECOND LOOP----------------"<<std::endl;
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_TRUE(qp->solve());
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
