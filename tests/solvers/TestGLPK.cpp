#include <gtest/gtest.h>
#include <OpenSoT/solvers/GLPKBackEnd.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/solvers/BackEndFactory.h> 
#include <chrono>
#include <OpenSoT/utils/AffineUtils.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/GenericLPTask.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <matlogger2/matlogger2.h>

#include "../common.h"

#define VEL_LIMS M_PI_2/2.
#define VEL_LIMS2 M_PI_2/2.
#define LAMBDA 0.03
#define OR_GAIN 0.1
#define dT 0.003

#define ENABLE_ROS false

#if ENABLE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#endif

namespace {

class testGLPKProblem: public TestBase
{
protected:

    testGLPKProblem() : TestBase("coman")
    {



#if ENABLE_ROS
      int argc = 0;
      char **argv;
      ros::init(argc, argv, "glpk_test");
      n.reset(new ros::NodeHandle());
      pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000);

      KDL::Tree my_tree;
      if (!kdl_parser::treeFromUrdfModel(*_model_ptr->getUrdf(), my_tree)){
        ROS_ERROR("Failed to construct kdl tree");}
      rsp.reset(new robot_state_publisher::RobotStatePublisher(my_tree));
      n->setParam("/robot_description", _model_ptr->getUrdfString());

      for(unsigned int i = 0; i < this->_model_ptr->getJointNames().size(); ++i){
          joint_state.name.push_back(this->_model_ptr->getJointNames()[i]);
          joint_state.position.push_back(0.0);}
#endif
    }

    virtual ~testGLPKProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void setGoodInitialPosition(Eigen::VectorXd& _q) {
        _q = _model_ptr->getNeutralQ();
        _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")] =  45.0*M_PI/180.;//20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")] = 0.0;//20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShYaw")] = 0.0;//-15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")] = -135.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    }

#if ENABLE_ROS
    void publishJointStates(const Eigen::VectorXd& q)
    {
        std::map<std::string, double> joint_map;
        for(unsigned int i = 0; i < q.size(); ++i){
            joint_state.position[i] = q[i];
            joint_map[joint_state.name[i]] = joint_state.position[i];
        }
        joint_state.header.stamp = ros::Time::now();

        pub.publish(joint_state);

        rsp->publishTransforms(joint_map, ros::Time::now(), "");
        rsp->publishFixedTransforms("");

        ros::spinOnce();
    }
#endif


#if ENABLE_ROS
  std::shared_ptr<ros::NodeHandle> n;
  ros::Publisher pub;
  sensor_msgs::JointState joint_state;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp;
#endif

};

XBot::MatLogger2::Ptr getLogger(const std::string& name)
{
    XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger(name); // date-time automatically appended
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    return logger;
}


TEST_F(testGLPKProblem, testIKMILP)
{
    std::cout<<"        FIRST RUN"<<std::endl;
/////////////////////////////////QP-QP-QP

    XBot::MatLogger2::Ptr log1 = getLogger("testIKMILP1");

    Eigen::VectorXd q;
    setGoodInitialPosition(q);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

#if ENABLE_ROS
    for(unsigned int i = 0; i<10; ++i)
    {
        this->publishJointStates(q);
        usleep(100000);
    }
    sleep(1);
#endif



    OpenSoT::OptvarHelper::VariableVector vars;
    vars.emplace_back("dq", _model_ptr->getNv());

    OpenSoT::OptvarHelper opt(vars);

    OpenSoT::tasks::velocity::Cartesian::Ptr RFoot =
        std::make_shared<OpenSoT::tasks::velocity::Cartesian>("base_Rfoot", *_model_ptr, "l_sole", "r_sole");
    RFoot->setLambda(LAMBDA);
    RFoot->setOrientationErrorGain(OR_GAIN);

    OpenSoT::tasks::velocity::Cartesian::Ptr LArm =
        std::make_shared<OpenSoT::tasks::velocity::Cartesian>("eeL", *_model_ptr, "LWrMot3", "l_sole");
    LArm->setLambda(LAMBDA);
    LArm->setOrientationErrorGain(OR_GAIN);

    OpenSoT::tasks::velocity::Postural::Ptr postural;
    postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);
    postural->setLambda(0.0);

    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims;
    joint_lims = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr,qmax, qmin);

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims;
    vel_lims = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, VEL_LIMS, dT);

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims_p;
    vel_lims_p = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, VEL_LIMS2, dT);

    using namespace OpenSoT::AffineUtils;
    OpenSoT::AutoStack::Ptr autostack = ((AffineTask::toAffine(RFoot, opt.getVariable("dq"))<<AffineConstraint::toAffine(vel_lims,opt.getVariable("dq")))
                                         / (AffineTask::toAffine(LArm, opt.getVariable("dq"))<<AffineConstraint::toAffine(vel_lims,opt.getVariable("dq")))
                                         / (postural<<AffineConstraint::toAffine(vel_lims_p,opt.getVariable("dq")))
                                         )<<AffineConstraint::toAffine(joint_lims,opt.getVariable("dq"));

    OpenSoT::solvers::iHQP::Ptr solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6,
                                                                                    OpenSoT::solvers::solver_back_ends::qpOASES);


    KDL::Frame ref;
    LArm->getReference(ref);
    ref.M.DoRotZ(-M_PI_2);
    LArm->setReference(ref);

    KDL::Frame foot_ref;
    RFoot->getReference(foot_ref);


    int t = 5000;
    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero(dq.size());
    Eigen::VectorXd solve_time(t);
    for(unsigned int i = 0; i < t; ++i)
    {
        this->_model_ptr->setJointPosition(q);
        this->_model_ptr->update();

//        Eigen::MatrixXd M;
//        this->_model_ptr->getInertiaMatrix(M);
//        postural->setWeight(M);

        autostack->update();

        auto tic = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve(dq));
        auto toc = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
        solve_time[i] = duration;

        q = _model_ptr->sum(q, dq);

        log1->add("solve_time", duration);
        log1->add("q", q);
        log1->add("dq", dq);
        autostack->log(log1);

#if ENABLE_ROS
        this->publishJointStates(q);
        usleep(100);
#endif
    }

    KDL::Frame LArm_pose;
    LArm->getActualPose(LArm_pose);

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(LArm_pose.p[i], ref.p[i], 1e-6);

    Eigen::Vector3d refRPY;
    ref.M.GetRPY(refRPY[0],refRPY[1],refRPY[2]);

    Eigen::Vector3d poseRPY;
    LArm_pose.M.GetRPY(poseRPY[0],poseRPY[1],poseRPY[2]);


    for(unsigned int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(poseRPY[i], refRPY[i], 1e-3);
    }
    KDL::Frame RFoot_pose;
    RFoot->getActualPose(RFoot_pose);

    EXPECT_TRUE(RFoot_pose.p == foot_ref.p);
    EXPECT_TRUE(RFoot_pose.M == foot_ref.M);




/////////////////////////////////QP-QP-MILP

    XBot::MatLogger2::Ptr log2 = getLogger("testIKMILP2");

std::cout<<"        SECOND RUN"<<std::endl;
    setGoodInitialPosition(q);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

#if ENABLE_ROS
    this->publishJointStates(q);
    this->publishJointStates(q);
    this->publishJointStates(q);


    sleep(1);
#endif

    OpenSoT::OptvarHelper::VariableVector vars2;
    vars2.emplace_back("dq", _model_ptr->getNv());
    vars2.emplace_back("delta", _model_ptr->getNv());

    OpenSoT::OptvarHelper opt2(vars2);

    OpenSoT::tasks::velocity::Cartesian::Ptr RFoot2 =
        std::make_shared<OpenSoT::tasks::velocity::Cartesian>("base_Rfoot", *_model_ptr, "l_sole", "r_sole");
    RFoot2->getReference(foot_ref);
    RFoot2->setLambda(LAMBDA);
    RFoot2->setOrientationErrorGain(OR_GAIN);


    OpenSoT::tasks::velocity::Cartesian::Ptr LArm2 =
        std::make_shared<OpenSoT::tasks::velocity::Cartesian>("eeL",*_model_ptr, "LWrMot3", "l_sole");
    LArm2->getReference(ref);
    ref.M.DoRotZ(-M_PI_2);
    LArm2->setReference(ref);
    LArm2->setLambda(LAMBDA);
    LArm2->setOrientationErrorGain(OR_GAIN);

    OpenSoT::constraints::velocity::JointLimits::Ptr joint_lims2;
    joint_lims2 = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr,qmax, qmin);


    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims2;
    vel_lims2 = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, VEL_LIMS, dT);


    OpenSoT::tasks::GenericLPTask::Ptr milp_task;
    Eigen::VectorXd c(_model_ptr->getNv());
    c.setOnes();
    milp_task = std::make_shared<OpenSoT::tasks::GenericLPTask>("milp_task",c, opt2.getVariable("delta"));

    double M =  VEL_LIMS2*dT + 1.*dT;
    double m = -M;
    double inf = 1e20;
    Eigen::MatrixXd a1(_model_ptr->getNv(),2*_model_ptr->getNv());
    a1<<Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv()), -M*Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv());

    Eigen::MatrixXd a2(_model_ptr->getNv(),2*_model_ptr->getNv());
    a2<<-1.0*Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv()), m*Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv());

    Eigen::MatrixXd A(2*_model_ptr->getNv(), 2*_model_ptr->getNv());
    A<<a1,
       a2;
    OpenSoT::AffineHelper tmp(A, Eigen::VectorXd::Zero(2*_model_ptr->getNv()));
    EXPECT_TRUE(A == tmp.getM());
    EXPECT_TRUE(tmp.getq() == Eigen::VectorXd::Zero(2*_model_ptr->getNv()));

    Eigen::VectorXd u(2*_model_ptr->getNv());
    u<<Eigen::VectorXd::Zero(_model_ptr->getNv()),Eigen::VectorXd::Zero(_model_ptr->getNv());

    Eigen::VectorXd l(2*_model_ptr->getNv());
    l<<-inf*Eigen::VectorXd::Ones(_model_ptr->getNv()), -inf*Eigen::VectorXd::Ones(_model_ptr->getNv());

    OpenSoT::constraints::GenericConstraint::Ptr milp_condition =
            std::make_shared<OpenSoT::constraints::GenericConstraint>("milp_condition",tmp,u,l,
        OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT);

    Eigen::VectorXd L(2*_model_ptr->getNv()), U(2*_model_ptr->getNv());
    L<<-VEL_LIMS2*dT*Eigen::VectorXd::Ones(_model_ptr->getNv()),     Eigen::VectorXd::Zero(_model_ptr->getNv());
    U<< VEL_LIMS2*dT*Eigen::VectorXd::Ones(_model_ptr->getNv()),     Eigen::VectorXd::Ones(_model_ptr->getNv());

    OpenSoT::constraints::GenericConstraint::Ptr milp_bounds = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                          "milp_bounds", U, L, 2*_model_ptr->getNv());


    OpenSoT::AutoStack::Ptr autostack2 = ((AffineTask::toAffine(RFoot2, opt2.getVariable("dq"))<<AffineConstraint::toAffine(vel_lims2,opt2.getVariable("dq")))
                                          / (AffineTask::toAffine(LArm2, opt2.getVariable("dq"))<<AffineConstraint::toAffine(vel_lims2,opt2.getVariable("dq")))
                                          / (milp_task<<milp_bounds<<milp_condition)
                                          )<<AffineConstraint::toAffine(joint_lims2,opt2.getVariable("dq"));

    OpenSoT::solvers::solver_back_ends solver_1 = OpenSoT::solvers::solver_back_ends::qpOASES;
    OpenSoT::solvers::solver_back_ends solver_2 = OpenSoT::solvers::solver_back_ends::qpOASES;
    OpenSoT::solvers::solver_back_ends solver_3 = OpenSoT::solvers::solver_back_ends::GLPK;

    std::vector<OpenSoT::solvers::solver_back_ends> solver_vector = {solver_1, solver_2, solver_3};

    OpenSoT::solvers::iHQP::Ptr solver2 = std::make_shared<OpenSoT::solvers::iHQP>(autostack2->getStack(), autostack2->getBounds(), 1e6,
                                                                                    solver_vector);

    OpenSoT::solvers::GLPKBackEnd::GLPKBackEndOptions optGLPK;
    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
        optGLPK.var_id_kind_.push_back(std::pair<int, int>(_model_ptr->getNv()+i, GLP_IV));

    OpenSoT::solvers::BackEnd::Ptr GLPK;
    solver2->getBackEnd(2,GLPK);
    GLPK->setOptions(optGLPK);


    Eigen::VectorXd dx(2*_model_ptr->getNv()), delta(_model_ptr->getNv());
    dx.setZero();
    dq.setZero();
    delta.setZero();
    OpenSoT::AffineHelper dqVar = opt2.getVariable("dq");
    OpenSoT::AffineHelper deltaVar = opt2.getVariable("delta");
    Eigen::VectorXd solve_time2(t);
    for(unsigned int i = 0; i < t; ++i)
    {
        this->_model_ptr->setJointPosition(q);
        this->_model_ptr->update();

        autostack2->update();

        auto tic = std::chrono::steady_clock::now();
        bool a = solver2->solve(dx);
        auto toc = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
        solve_time2[i] = duration;


        dqVar.getValue(dx, dq);
        deltaVar.getValue(dx, delta);


        solver2->log(log2);



        if(a){
            q = _model_ptr->sum(q, dq);
            //std::cout<<"dq: "<<dq.transpose()<<std::endl;
            //std::cout<<"delta: "<<delta.transpose()<<std::endl;
        }

        log2->add("solve_time", duration);
        log2->add("q", q);
        log2->add("dq", dq);
        log2->add("delta", delta);
        autostack2->log(log2);

#if ENABLE_ROS
        this->publishJointStates(q);
        usleep(100);
#endif
    }


    LArm2->getActualPose(LArm_pose);

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(LArm_pose.p[i], ref.p[i], 1e-6);

    ref.M.GetRPY(refRPY[0],refRPY[1],refRPY[2]);

    LArm_pose.M.GetRPY(poseRPY[0],poseRPY[1],poseRPY[2]);


    for(unsigned int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(poseRPY[i], refRPY[i], 1e-3);
    }


    RFoot2->getActualPose(RFoot_pose);

    EXPECT_TRUE(RFoot_pose.p == foot_ref.p);
    EXPECT_TRUE(RFoot_pose.M == foot_ref.M);



    double media_solve_time = solve_time.sum()/solve_time.size();
    double media_solve_time2 = solve_time2.sum()/solve_time2.size();

    std::cout<<"iHQP media solve time: "<<media_solve_time<<std::endl;
    std::cout<<"MILP-IK media solve time: "<<media_solve_time2<<std::endl;
}

TEST_F(testGLPKProblem, testMILPProblem)
{
    Eigen::MatrixXd A(3,3); A.setZero(3,3);
    Eigen::VectorXd b(3); b.setZero(3);
    OpenSoT::tasks::GenericTask::Ptr task(new OpenSoT::tasks::GenericTask("task",A,b));
    Eigen::VectorXd c(3);
    c<<3.,-2.,-1;
    task->setHessianType(OpenSoT::HST_ZERO);
    task->setc(c);
    task->update();

    Eigen::VectorXd lb(3), ub(3);
    lb.setZero(3);
    ub<<1e30, 1e30, 1.;
    OpenSoT::constraints::GenericConstraint::Ptr bounds(new OpenSoT::constraints::GenericConstraint("bounds", ub, lb, 3));
    bounds->update();

    Eigen::MatrixXd Ac(2,3);
    Ac<<1.,1.,1.,
        4.,2.,1.;
    Eigen::VectorXd lA(2), uA(2);
    lA<<-1e30, 12.;
    uA<<7., 12.;

    OpenSoT::AffineHelper var(Ac, Eigen::VectorXd::Zero(2));
    OpenSoT::constraints::GenericConstraint::Ptr constr(new OpenSoT::constraints::GenericConstraint("constraint", var, uA, lA, OpenSoT::constraints::GenericConstraint::Type::CONSTRAINT));
    constr->update();
    std::cout<<"lA: "<<constr->getbLowerBound();

    OpenSoT::solvers::GLPKBackEnd::Ptr solver = OpenSoT::solvers::BackEndFactory(
                OpenSoT::solvers::solver_back_ends::GLPK, 3, 2, task->getHessianAtype(), 0.0);



    auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(solver->initProblem(Eigen::MatrixXd(0,0), task->getc(),
                       constr->getAineq(), constr->getbLowerBound(), constr->getbUpperBound(),
                       bounds->getLowerBound(), bounds->getUpperBound()));
    auto stop = std::chrono::steady_clock::now();
    auto time_for_initProblem = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
    std::cout<<"Time elapsed for time_for_initProblem: "<<time_for_initProblem/1000.<<" [ms]"<<std::endl;
    std::cout<<"Solution from initProblem: \n"<<solver->getSolution()<<std::endl;

    std::vector<double> times;
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0],0.0, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1],6., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2],0.0, 1e-12);

    ub[1] = 5;
    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0],0.25, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1],5., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2],1.0, 1e-12);

    ub[1] = 9.1;

    OpenSoT::solvers::GLPKBackEnd::GLPKBackEndOptions opt;
    opt.var_id_kind_.push_back(std::pair<int,int>(1,GLP_IV));
    opt.var_id_kind_.push_back(std::pair<int,int>(2,GLP_BV));
    opt.ROUND_BOUNDS = -1;
    solver->setOptions(opt);
    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 0., 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], 6., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 0., 1e-12);

    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 0., 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], 6., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 0., 1e-12);

    lb[1] = -2.1;
    ub[1] = -1.;

    solver->updateBounds(lb, ub);
    for(unsigned int i = 0; i<10; ++i)
    {
        start = std::chrono::steady_clock::now();
        EXPECT_TRUE(solver->solve());
        stop = std::chrono::steady_clock::now();
        std::cout<<"Solution from solve: \n"<<solver->getSolution()<<std::endl;

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start).count();
        times.push_back(duration);
    }

    EXPECT_NEAR(solver->getSolution()[0], 3.25, 1e-12);
    EXPECT_NEAR(solver->getSolution()[1], -1., 1e-12);
    EXPECT_NEAR(solver->getSolution()[2], 1., 1e-12);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

