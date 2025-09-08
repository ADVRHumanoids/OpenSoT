#include <gtest/gtest.h>
#include <OpenSoT/solvers/qpSWIFTBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <qpSWIFT.h>
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
        A.setIdentity(A.rows(), A.cols());
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

class testqpSWIFTProblem: public TestBase
{
protected:

    testqpSWIFTProblem():TestBase("coman_floating_base")
    {

    }

    virtual ~testqpSWIFTProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testqpSWIFTProblem, testOptions)
{


    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::BackEnd::Ptr testProblemqpSWIFT = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpSWIFT, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    EXPECT_TRUE(testProblemqpSWIFT->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));


    auto plr = boost::any_cast<settings*>(testProblemqpSWIFT->getOptions());
    std::cout<<"plr->maxit: "<<plr->maxit<<std::endl;
    std::cout<<"plr->reltol: "<<plr->reltol<<std::endl;
    std::cout<<"plr->abstol: "<<plr->abstol<<std::endl;
    std::cout<<"plr->sigma: "<<plr->sigma<<std::endl;
    std::cout<<"plr->verbose: "<<plr->verbose<<std::endl;



    std::shared_ptr<settings> opt = std::make_shared<settings>();
    opt->maxit = 10;
    opt->reltol = 1.;
    opt->abstol = 2.;
    opt->sigma = 3.;
    opt->verbose = 2;

    testProblemqpSWIFT->setOptions(opt.get());

    auto tlr = boost::any_cast<settings*>(testProblemqpSWIFT->getOptions());

    std::cout<<"tlr->maxit: "<<tlr->maxit<<std::endl;
    std::cout<<"tlr->reltol: "<<tlr->reltol<<std::endl;
    std::cout<<"tlr->abstol: "<<tlr->abstol<<std::endl;
    std::cout<<"tlr->sigma: "<<tlr->sigma<<std::endl;
    std::cout<<"tlr->verbose: "<<tlr->verbose<<std::endl;

    EXPECT_EQ(tlr->maxit, opt->maxit);
    EXPECT_EQ(tlr->reltol, opt->reltol);
    EXPECT_EQ(tlr->abstol, opt->abstol);
    EXPECT_EQ(tlr->sigma, opt->sigma);
    EXPECT_EQ(tlr->verbose, opt->verbose);

}


TEST_F(testqpSWIFTProblem, testSimpleProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::BackEnd::Ptr testProblemQPOASES = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    OpenSoT::solvers::BackEnd::Ptr testProblemqpSWIFT = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpSWIFT, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    EXPECT_TRUE(testProblemQPOASES->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    EXPECT_TRUE(testProblemqpSWIFT->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
    Eigen::VectorXd s_qpswift = testProblemqpSWIFT->getSolution();

    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution qpSWIFT: "<<s_qpswift.transpose()<<std::endl;

    EXPECT_NEAR((s_qpoases - s_qpswift).norm(),0.0, 1e-10);
    std::cout<<"Second solve"<<std::endl;

    EXPECT_TRUE(testProblemQPOASES->solve());
    EXPECT_TRUE(testProblemqpSWIFT->solve());
    s_qpoases = testProblemQPOASES->getSolution();
    s_qpswift = testProblemqpSWIFT->getSolution();
    EXPECT_NEAR(-sp.g[0], s_qpswift[0], 1e-10);
    EXPECT_NEAR(-sp.g[1], s_qpswift[1], 1e-10);
    EXPECT_NEAR((s_qpoases - s_qpswift).norm(),0.0, 1e-10);
    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution qpSWIFT: "<<s_qpswift.transpose()<<std::endl;

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblemqpSWIFT->solve());
        EXPECT_TRUE(testProblemQPOASES->solve());

        Eigen::VectorXd s_qpswift = testProblemqpSWIFT->getSolution();
        Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
        EXPECT_NEAR(-sp.g[0], s_qpswift[0], 1e-10);
        EXPECT_NEAR(-sp.g[1], s_qpswift[1], 1e-10);
        EXPECT_NEAR((s_qpoases - s_qpswift).norm(),0.0, 1e-10);
    }

}


TEST_F(testqpSWIFTProblem, testTask)
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
                OpenSoT::solvers::solver_back_ends::qpSWIFT, postural_task.getXSize(), 0,
                OpenSoT::HST_IDENTITY,0.);

    Eigen::VectorXd q_max(_model_ptr->getNv()), q_min(_model_ptr->getNv());
    q_max.setConstant(1000.);
    q_min.setConstant(-1000.);


    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits =
            std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr, q_max, q_min);


    qp_postural_problem->initProblem(H,g,Eigen::MatrixXd(0,0),Eigen::VectorXd(),Eigen::VectorXd(),joint_limits->getLowerBound(),joint_limits->getUpperBound());

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

        std::cout<<"q: "<<q.transpose()<<std::endl;
    }

    q = _model_ptr->sum(q, dq);
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i], q_ref[i], 1E-12);


    std::cout<<"q: "<<q.transpose()<<std::endl;
    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
}

using namespace OpenSoT::constraints::velocity;
TEST_F(testqpSWIFTProblem, testProblemWithConstraint)
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
        postural_task->update();

        OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                    OpenSoT::solvers::solver_back_ends::qpSWIFT, postural_task->getXSize(), 0,
                    OpenSoT::HST_IDENTITY,0.);

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
            postural_task->update();

            qp_postural_problem->updateBounds(constraint->getLowerBound(), constraint->getUpperBound());
            qp_postural_problem->updateTask(postural_task->getA(), -1.0*postural_task->getb());

            EXPECT_TRUE(qp_postural_problem->solve());
            l = qp_postural_problem->getl();
            u = qp_postural_problem->getu();
            Eigen::VectorXd dq = qp_postural_problem->getSolution();
            q = _model_ptr->sum(q,dq);

            _model_ptr->setJointPosition(q);
            _model_ptr->update();

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 7; i < q.size(); ++i)
        {
            std::cout<<q_min[i-1]<<" <= "<<q[i]<<" <= "<<q_max[i-1]<<std::endl;
            if(q_ref[i] >= q_max[i-1])
            {
                std::cout<<"On the Upper Bound!"<<std::endl;
                EXPECT_NEAR( q[i], q_max(i-1), 1E-4);
            }
            else if(q_ref[i] <= q_min[i-1])
            {
                std::cout<<"On the Lower Bound!"<<std::endl;
                EXPECT_NEAR( q[i], q_min[i-1], 1E-4);
            }
            else
                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
        }
}

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();

    _q[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testqpSWIFTProblem, testCartesian)
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

    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpSWIFT, cartesian_task->getXSize(), 0,
                OpenSoT::HST_SEMIDEF,0.);

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

TEST_F(testqpSWIFTProblem, testEpsRegularisation)
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

    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpSWIFT, cartesian_task->getXSize(), 0,
                OpenSoT::HST_SEMIDEF,0.);

    EXPECT_EQ(qp_cartesian_problem->getEpsRegularisation(), 0.);

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

TEST_F(testqpSWIFTProblem, testContructor2Problems)
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
        new JointLimits(*_model_ptr, q_max, q_min));
    joint_limits->setBoundScaling((double)(1.0/t));

    VelocityLimits::Ptr joint_velocity_limits(new VelocityLimits(*_model_ptr, 1.0, (double)(1.0/t)));

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

    OpenSoT::solvers::iHQP sot(stack_of_tasks, joint_constraints, 0., OpenSoT::solvers::solver_back_ends::qpSWIFT);


    KDL::Frame T_ref_kdl;
    T_ref_kdl.p[0] = 0.283; T_ref_kdl.p[1] = 0.156; T_ref_kdl.p[2] = 0.499;
    T_ref_kdl.M = T_ref_kdl.M.Quaternion(0.0, 0.975, 0.0, -0.221);
    Eigen::Affine3d T_ref;
    tf2::transformKDLToEigen(T_ref_kdl, T_ref);
    cartesian_task->setReference(T_ref);

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
    Eigen::Affine3d T_kdl;
    _model_ptr->getPose("l_wrist", "Waist", T_kdl);
    std::cout<<"FINAL CONFIG: "<<T_kdl.matrix()<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref.matrix()<<std::endl;


    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(T_kdl.translation()[i], T_ref.translation()[i], 1E-3);
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(T_kdl.linear()(i,j), T_ref.linear()(i,j), 1E-2);


}

class testiHQP: public TestBase
{
protected:

    testiHQP():TestBase("coman_floating_base")
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
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds, 1e-6, OpenSoT::solvers::solver_back_ends::qpSWIFT);
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
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


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

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
