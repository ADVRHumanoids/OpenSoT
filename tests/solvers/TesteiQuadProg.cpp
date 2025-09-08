#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include "../common.h"


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

class testeiQuadProgProblem: public TestBase
{
protected:

    testeiQuadProgProblem():TestBase("coman_floating_base")
    {

    }

    virtual ~testeiQuadProgProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testeiQuadProgProblem, testSimpleProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::BackEnd::Ptr testProblemQPOASES = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    OpenSoT::solvers::BackEnd::Ptr testProblemeiQuadProg = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    EXPECT_TRUE(testProblemQPOASES->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    EXPECT_TRUE(testProblemeiQuadProg->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
    Eigen::VectorXd s_eiQuadProg = testProblemeiQuadProg->getSolution();

    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution eiQuadProg: "<<s_eiQuadProg.transpose()<<std::endl;

    std::cout<<"QPOASES: 0.5*x'Hx + gx: "<<0.5*s_qpoases.transpose()*sp.H*s_qpoases + sp.g.transpose()*s_qpoases<<std::endl;
    std::cout<<"eiQuadProg: 0.5*x'Hx + gx: "<<0.5*s_eiQuadProg.transpose()*sp.H*s_eiQuadProg + sp.g.transpose()*s_eiQuadProg<<std::endl;

    EXPECT_NEAR((s_qpoases - s_eiQuadProg).norm(),0.0, 1e-12);

    EXPECT_TRUE(testProblemQPOASES->solve());
    EXPECT_TRUE(testProblemeiQuadProg->solve());
    s_qpoases = testProblemQPOASES->getSolution();
    s_eiQuadProg = testProblemeiQuadProg->getSolution();
    EXPECT_EQ(-sp.g[0], s_eiQuadProg[0]);
    EXPECT_EQ(-sp.g[1], s_eiQuadProg[1]);
    EXPECT_NEAR((s_qpoases - s_eiQuadProg).norm(),0.0, 1e-12);
    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution aiQuadProg: "<<s_eiQuadProg.transpose()<<std::endl;

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblemeiQuadProg->solve());
        EXPECT_TRUE(testProblemQPOASES->solve());

        Eigen::VectorXd s_eiQuadProg = testProblemeiQuadProg->getSolution();
        Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
        EXPECT_NEAR(-sp.g[0], s_eiQuadProg[0], 1e-12);
        EXPECT_NEAR(-sp.g[1], s_eiQuadProg[1], 1e-12);
        EXPECT_NEAR((s_qpoases - s_eiQuadProg).norm(),0.0, 1e-12);
    }

}


TEST_F(testeiQuadProgProblem, testTask)
{
    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();
    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    std::cout<<"q: "<<q.transpose()<<std::endl;


    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural postural_task(*_model_ptr);
    postural_task.setReference(q_ref);
    postural_task.update();

    Eigen::MatrixXd H(_model_ptr->getNv(), _model_ptr->getNv()); H.setIdentity(H.rows(), H.cols());
    Eigen::VectorXd g(-1.0*postural_task.getb());

    OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, postural_task.getXSize(), 0,
                OpenSoT::HST_IDENTITY,0.);

    qp_postural_problem->initProblem(H,g,Eigen::MatrixXd(0,0),Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd());

    Eigen::VectorXd dq = qp_postural_problem->getSolution();
    std::cout<<"dq: "<<dq.transpose()<<std::endl;

    for(unsigned int i = 0; i < 10; ++i)
    {
        q = _model_ptr->sum(q, dq);

        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task.update();

        qp_postural_problem->updateTask(H, -1.0*postural_task.getb());

        EXPECT_TRUE(qp_postural_problem->solve());
        dq = qp_postural_problem->getSolution();

        std::cout<<"dq: "<<dq.transpose()<<std::endl;
    }
    q = _model_ptr->sum(q, dq);

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i], q_ref[i], 1E-12);


    std::cout<<"q: "<<q.transpose()<<std::endl;
    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
}

using namespace OpenSoT::constraints::velocity;
TEST_F(testeiQuadProgProblem, testProblemWithConstraint)
{

    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();
    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    std::cout<<"q: "<<q.transpose()<<std::endl;

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
    postural_task->setLambda(0.1);

    OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, postural_task->getXSize(), 0,
                OpenSoT::HST_IDENTITY,0.);

    std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraint_list =
                postural_task->getConstraints();
    OpenSoT::Constraint< Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr constraint = constraint_list.front();
    EXPECT_TRUE(qp_postural_problem->initProblem(postural_task->getA(), -1.0*postural_task->getb(),
                                                    Eigen::MatrixXd(), Eigen::VectorXd(), Eigen::VectorXd(),
                                                    constraint->getLowerBound(), constraint->getUpperBound()));

    Eigen::VectorXd l_old = qp_postural_problem->getl();
    Eigen::VectorXd u_old = qp_postural_problem->getu();


    Eigen::VectorXd l, u;
    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update();

        qp_postural_problem->updateBounds(constraint->getLowerBound(), constraint->getUpperBound());
        qp_postural_problem->updateTask(postural_task->getA(), -1.0*postural_task->getb());

        EXPECT_TRUE(qp_postural_problem->solve());
        l = qp_postural_problem->getl();
        u = qp_postural_problem->getu();
        Eigen::VectorXd dq = qp_postural_problem->getSolution();
        q = _model_ptr->sum(q,dq);

        if(i > 1)
        {
            EXPECT_FALSE(l == l_old);
            EXPECT_FALSE(u == u_old);
        }
    }

    auto q_min_ = q_min.segment(6,q_min.size()-6);
    auto q_max_ = q_max.segment(6,q_max.size()-6);
    auto q_ = q.segment(7,q.size()-7);
    auto q_ref_ = q_ref.segment(7,q.size()-7);

    for(unsigned int i = 0; i < q_.size(); ++i)
    {
        std::cout<<q_min_[i]<<" <= "<<q_[i]<<" <= "<<q_max_[i]<<std::endl;
        if(q_ref_[i] >= q_max_[i])
        {
            std::cout<<"On the Upper Bound!"<<std::endl;
            EXPECT_NEAR( q_[i], q_max_[i], 1E-4);
        }
        else if(q_ref_[i] <= q_min_[i])
        {
            std::cout<<"On the Lower Bound!"<<std::endl;
            EXPECT_NEAR( q_[i], q_min_[i], 1E-4);
        }
        else
            EXPECT_NEAR( q_[i], q_ref_[i], 1E-4);
    }
}


Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();
    _q[_model_ptr->getDofIndex("RHipSag") + 1] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag") + 1] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag") + 1] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag") + 1] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag") + 1] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag") + 1] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag") + 1] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat") + 1] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj") + 1] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag") + 1] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat") + 1] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj") + 1] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testeiQuadProgProblem, testCartesian)
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

    cartesian_task->update();

    Eigen::MatrixXd T_actual = cartesian_task->getActualPose();
    std::cout<<"T_actual: \n"<<T_actual<<std::endl;


    Eigen::MatrixXd T_ref = T.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.02;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    cartesian_task->setReference(T_ref);
    cartesian_task->update();

    //OpenSoT::solvers::OSQPBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0);
    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, cartesian_task->getXSize(), 0,
                OpenSoT::HST_SEMIDEF,1e6);

    ASSERT_TRUE(qp_cartesian_problem->initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.001*Eigen::VectorXd::Ones(_model_ptr->getNv()), 0.001*Eigen::VectorXd::Ones(_model_ptr->getNv())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update();
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

TEST_F(testeiQuadProgProblem, testEpsRegularisation)
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

    cartesian_task->update();

    Eigen::MatrixXd T_actual = cartesian_task->getActualPose();
    std::cout<<"T_actual: \n"<<T_actual<<std::endl;


    Eigen::MatrixXd T_ref = T.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.02;
    std::cout<<"T_ref: \n"<<T_ref<<std::endl;

    cartesian_task->setReference(T_ref);
    cartesian_task->update();

    //OpenSoT::solvers::OSQPBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0);
    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, cartesian_task->getXSize(), 0,
                OpenSoT::HST_SEMIDEF,1e6);

    EXPECT_NEAR(qp_cartesian_problem->getEpsRegularisation(), 2.22e-07, 1e-6);
    std::cout<<"qp_cartesian_problem->getEpsRegularisation(): "<<qp_cartesian_problem->getEpsRegularisation()<<std::endl;

    qp_cartesian_problem->setEpsRegularisation(1e-4);
    EXPECT_EQ(qp_cartesian_problem->getEpsRegularisation(), 1e-4);

    ASSERT_TRUE(qp_cartesian_problem->initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.001*Eigen::VectorXd::Ones(_model_ptr->getNv()), 0.001*Eigen::VectorXd::Ones(_model_ptr->getNv())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update();
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

TEST_F(testeiQuadProgProblem, testContructor2Problems)
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
    cartesian_task->setLambda(0.1);
    postural_task->setLambda(0.1);

    postural_task->setReference(q);
    cartesian_task->setReference(T_init.matrix());

    int t = 100;
    //Constraints set to the Cartesian Task
    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits(q_min, q_max);

    std::cout<<"q_min: ["<<q_min.transpose()<<"]"<<std::endl;
    std::cout<<"q_max: ["<<q_max.transpose()<<"]"<<std::endl;

    JointLimits::Ptr joint_limits(
        new JointLimits(*_model_ptr, q_max,
                           q_min));
    joint_limits->setBoundScaling((double)(1.0/t));

    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(*_model_ptr, 1.0, (double)(1.0/t)));

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

    OpenSoT::solvers::iHQP sot(stack_of_tasks, joint_constraints, 1e6, OpenSoT::solvers::solver_back_ends::eiQuadProg);


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

        cartesian_task->update();
        postural_task->update();
        joint_constraints->update();

        ASSERT_TRUE(sot.solve(dq));
        //std::cout<<"Solution: ["<<dq<<"]"<<std::endl;
        q = _model_ptr->sum(q, dq);
    }

    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;
    Eigen::Affine3d T;
    _model_ptr->getPose("l_wrist", "Waist", T);

    KDL::Frame T_kdl;
    tf2::transformEigenToKDL(T, T_kdl);

    std::cout<<"FINAL CONFIG: "<<T.matrix()<<std::endl;
    Eigen::Affine3d T_ref;
    tf2::transformKDLToEigen(T_ref_kdl, T_ref);
    std::cout<<"DESIRED CONFIG: "<<T_ref.matrix()<<std::endl;


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.p[i], T_ref_kdl.p[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.M(i,j), T_ref_kdl.M(i,j), 1E-2);


}

class testiHQP: public TestBase
{
protected:

    testiHQP() : TestBase("coman_floating_base")
    {
    }

    virtual ~testiHQP() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

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
                new OpenSoT::constraints::Aggregated(bounds_list, _model_ptr->getNv()));

    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
    stack_of_tasks.push_back(postural_task);
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds, 1e-6, OpenSoT::solvers::solver_back_ends::eiQuadProg);
    double obj;
    sot.getObjective(0, obj);
    obj = norm(obj);
    std::cout<<"obj: "<<obj<<std::endl;

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    double obj_;
    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update();
        bounds->update();

        EXPECT_TRUE(sot.solve(dq));
        sot.getObjective(0,obj_);
        obj_ = norm(obj_);
        if(i > 0)
        {
            if(obj_ > 1e-9)
                EXPECT_LE(obj_,obj);
        }
        obj = obj_;
        q = _model_ptr->sum(q, dq);

        std::cout<<"obj: "<<obj<<std::endl;
    }

    auto q_min_ = q_min.segment(6,q_min.size()-6);
    auto q_max_ = q_max.segment(6,q_max.size()-6);
    auto q_ = q.segment(7,q.size()-7);
    auto q_ref_ = q_ref.segment(7,q.size()-7);


    for(unsigned int i = 0; i < q_.size(); ++i)
    {
        if(q_ref_[i] >= q_max_[i])
        {
            std::cout<<GREEN<<"On the Upper Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q_[i], q_max_[i], 1E-4);
        }
        else if(q_ref_[i] <= q_min_[i])
        {
            std::cout<<GREEN<<"On the Lower Bound!"<<DEFAULT<<std::endl;
            EXPECT_NEAR( q_[i], q_min_[i], 1E-4);
        }
        else
            EXPECT_NEAR( q_[i], q_ref_[i], 1E-4);

    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
