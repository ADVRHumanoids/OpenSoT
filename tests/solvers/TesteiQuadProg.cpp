//#include <OpenSoT/solvers/uQuadProgBackEnd.h>
#include <OpenSoT/solvers/QPOasesBackEnd.h>
#include <qpOASES.hpp>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

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

class testuQuadProgProblem: public ::testing::Test
{
protected:

    testuQuadProgProblem()
    {

    }

    virtual ~testuQuadProgProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testuQuadProgProblem, testSimpleProblem)
{
    Eigen::VectorXd x(2);
    simpleProblem sp;

    OpenSoT::solvers::BackEnd::Ptr testProblemQPOASES = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::qpOASES, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    OpenSoT::solvers::BackEnd::Ptr testProblemuQuadProg = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, x.size(), sp.A.rows(), (OpenSoT::HessianType)sp.ht,0.);

    EXPECT_TRUE(testProblemQPOASES->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    EXPECT_TRUE(testProblemuQuadProg->initProblem(sp.H,
                            sp.g,
                            sp.A,
                            sp.lA,
                            sp.uA,
                            sp.l,
                            sp.u));

    Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
    Eigen::VectorXd s_uQuadProg = testProblemuQuadProg->getSolution();

    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution uQuadProg: "<<s_uQuadProg.transpose()<<std::endl;

    std::cout<<"QPOASES: 0.5*x'Hx + gx: "<<0.5*s_qpoases.transpose()*sp.H*s_qpoases + sp.g.transpose()*s_qpoases<<std::endl;
    std::cout<<"uQuadProg: 0.5*x'Hx + gx: "<<0.5*s_uQuadProg.transpose()*sp.H*s_uQuadProg + sp.g.transpose()*s_uQuadProg<<std::endl;

    EXPECT_NEAR((s_qpoases - s_uQuadProg).norm(),0.0, 1e-12);

    EXPECT_TRUE(testProblemQPOASES->solve());
    EXPECT_TRUE(testProblemuQuadProg->solve());
    s_qpoases = testProblemQPOASES->getSolution();
    s_uQuadProg = testProblemuQuadProg->getSolution();
    EXPECT_EQ(-sp.g[0], s_uQuadProg[0]);
    EXPECT_EQ(-sp.g[1], s_uQuadProg[1]);
    EXPECT_NEAR((s_qpoases - s_uQuadProg).norm(),0.0, 1e-12);
    std::cout<<"solution QPOASES: "<<s_qpoases.transpose()<<std::endl;
    std::cout<<"solution uQuadProg: "<<s_uQuadProg.transpose()<<std::endl;

    for(unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_TRUE(testProblemuQuadProg->solve());
        EXPECT_TRUE(testProblemQPOASES->solve());

        Eigen::VectorXd s_uQuadProg = testProblemuQuadProg->getSolution();
        Eigen::VectorXd s_qpoases = testProblemQPOASES->getSolution();
        EXPECT_NEAR(-sp.g[0], s_uQuadProg[0], 1e-12);
        EXPECT_NEAR(-sp.g[1], s_uQuadProg[1], 1e-12);
        EXPECT_NEAR((s_qpoases - s_uQuadProg).norm(),0.0, 1e-12);
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

TEST_F(testuQuadProgProblem, testTask)
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

    //OpenSoT::solvers::OSQPBackEnd qp_postural_problem(postural_task.getXSize(), 0);
    OpenSoT::solvers::BackEnd::Ptr qp_postural_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, postural_task.getXSize(), 0,
                OpenSoT::HST_IDENTITY,0.);

    qp_postural_problem->initProblem(H,g,Eigen::MatrixXd(0,0),Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd(),Eigen::VectorXd());

    Eigen::VectorXd dq = qp_postural_problem->getSolution();

    for(unsigned int i = 0; i < 10; ++i)
    {
        q += dq;
        postural_task.update(q);

        qp_postural_problem->updateTask(H, -1.0*postural_task.getb());

        EXPECT_TRUE(qp_postural_problem->solve());
        dq = qp_postural_problem->getSolution();

        std::cout<<"dq: "<<dq.transpose()<<std::endl;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR( q[i] + dq[i], q_ref[i], 1E-12);


    std::cout<<"q+dq: "<<(q + dq).transpose()<<std::endl;
    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
}

using namespace OpenSoT::constraints::velocity;
TEST_F(testuQuadProgProblem, testProblemWithConstraint)
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

        //OpenSoT::solvers::OSQPBackEnd qp_postural_problem(postural_task->getXSize(), 0);
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

        EXPECT_TRUE(l_old == q_min);
        EXPECT_TRUE(u_old == q_max);

        Eigen::VectorXd l, u;
        for(unsigned int i = 0; i < 100; ++i)
        {
            postural_task->update(q);

            qp_postural_problem->updateBounds(constraint->getLowerBound(), constraint->getUpperBound());
            qp_postural_problem->updateTask(postural_task->getA(), -1.0*postural_task->getb());

            EXPECT_TRUE(qp_postural_problem->solve());
            l = qp_postural_problem->getl();
            u = qp_postural_problem->getu();
            Eigen::VectorXd dq = qp_postural_problem->getSolution();
            q += dq;

            if(i > 1)
            {
                EXPECT_FALSE(l == l_old);
                EXPECT_FALSE(u == u_old);
            }
        }

        for(unsigned int i = 0; i < q.size(); ++i)
        {
            std::cout<<q_min[i]<<" <= "<<q[i]<<" <= "<<q_max[i]<<std::endl;
            if(q_ref[i] >= q_max[i])
            {
                std::cout<<"On the Upper Bound!"<<std::endl;
                EXPECT_NEAR( q[i], q_max(i), 1E-4);
            }
            else if(q_ref[i] <= q_min[i])
            {
                std::cout<<"On the Lower Bound!"<<std::endl;
                EXPECT_NEAR( q[i], q_min[i], 1E-4);
            }
            else
                EXPECT_NEAR( q[i], q_ref[i], 1E-4);
        }
}


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

TEST_F(testuQuadProgProblem, testCartesian)
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

    //OpenSoT::solvers::OSQPBackEnd qp_cartesian_problem(cartesian_task->getXSize(), 0);
    OpenSoT::solvers::BackEnd::Ptr qp_cartesian_problem = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::eiQuadProg, cartesian_task->getXSize(), 0,
                OpenSoT::HST_SEMIDEF,1e6);

    ASSERT_TRUE(qp_cartesian_problem->initProblem(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb(),
                                                Eigen::MatrixXd(0,0), Eigen::VectorXd(), Eigen::VectorXd(),
                                                -0.001*Eigen::VectorXd::Ones(q.size()), 0.001*Eigen::VectorXd::Ones(q.size())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(q);
        qp_cartesian_problem->updateTask(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem->solve());
        Eigen::VectorXd dq = qp_cartesian_problem->getSolution();
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

TEST_F(testuQuadProgProblem, testEpsRegularisation)
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
                                                -0.001*Eigen::VectorXd::Ones(q.size()), 0.001*Eigen::VectorXd::Ones(q.size())));

    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(q);
        qp_cartesian_problem->updateTask(cartesian_task->getA().transpose()*cartesian_task->getA(), -1.0*cartesian_task->getA().transpose()*cartesian_task->getb());
        ASSERT_TRUE(qp_cartesian_problem->solve());
        Eigen::VectorXd dq = qp_cartesian_problem->getSolution();
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

TEST_F(testuQuadProgProblem, testContructor2Problems)
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
        new JointLimits(q, q_max,
                           q_min));
    joint_limits->setBoundScaling((double)(1.0/t));

    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(1.0, (double)(1.0/t), q.size()));

    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

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

class testiHQP: public ::testing::Test
{
protected:

    testiHQP()
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
    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds, 1e-6, OpenSoT::solvers::solver_back_ends::eiQuadProg);
    double obj;
    sot.getObjective(0, obj);
    obj = norm(obj);
    std::cout<<"obj: "<<obj<<std::endl;

    EXPECT_TRUE(sot.getNumberOfTasks() == 1);
    Eigen::VectorXd dq(q.size());
    dq.setZero(q.size());
    double obj_;
    for(unsigned int i = 0; i < 100; ++i)
    {
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot.solve(dq));
        sot.getObjective(0,obj_);
        obj_ = norm(obj_);
        if(i > 0)
        {
            if(obj_ > 1e-9)
                EXPECT_LE(obj_,obj);
        }
        obj = obj_;
        Eigen::VectorXd _dq = dq;
        q += _dq;

        std::cout<<"obj: "<<obj<<std::endl;
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

/////THIS TESTS WORKS BUT DOES NOT TEST ANYTHING DURING THE EXECUTION, IT JUST LOG THE TWO SOLVERS.
////TEST_F(testiHQP, testMultipleSolversLogs)
////{
////    Eigen::VectorXd q(30);
////    q.setRandom(q.size());

////    Eigen::VectorXd q_ref(q.size());
////    q_ref.setZero(q_ref.size());

////    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
////            new OpenSoT::tasks::velocity::Postural(q));
////    postural_task->setReference(q_ref);
////    postural_task->update(q);


////    Eigen::VectorXd q_min(q_ref.size()), q_max(q_ref.size());
////    q_min = -10.*Eigen::VectorXd::Ones(q.size());
////    q_max = -q_min;

////    JointLimits::Ptr joint_limits(new JointLimits(q,q_max,q_min));
////    joint_limits->update(q);

////    OpenSoT::solvers::iHQP::Stack stack_of_tasks;
////    stack_of_tasks.push_back(postural_task);

////    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> bounds_list;
////        bounds_list.push_back(joint_limits);

////        OpenSoT::constraints::Aggregated::Ptr bounds(
////                    new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

////    OpenSoT::solvers::iHQP sot(stack_of_tasks, bounds, 1e-6, OpenSoT::solvers::solver_back_ends::OSQP);

////    OpenSoT::solvers::iHQP sot2(stack_of_tasks, bounds, 1e-6, OpenSoT::solvers::solver_back_ends::OSQP);
////    sot2.setSolverID("sot2");


////    XBot::MatLogger2::Ptr logger = XBot::MatLogger::getLogger("testMultipleSolversLogs");

////    sot.log(logger);
////    sot2.log(logger);

////    logger->flush();
////}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
