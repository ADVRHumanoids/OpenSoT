#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/utils/cartesian_utils.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/floating_base_estimation/qp_estimation.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/solvers/iHQP.h>

namespace {

class testKinematicEstimation: public ::testing::Test
{
public:
    XBot::ModelInterface::Ptr _model_ptr;
    XBot::ModelInterface::Ptr _model_ptr_kinematic_estimation;
    std::string _path_to_cfg;
    XBot::MatLogger::Ptr _logger;

    testKinematicEstimation()
    {
        std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
        std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";

        _path_to_cfg = robotology_root + relative_path;

        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
        _model_ptr_kinematic_estimation = XBot::ModelInterface::getModel(_path_to_cfg);

        _logger = XBot::MatLogger::getLogger("/tmp/testKinematicEstimation");
    }

    virtual ~testKinematicEstimation() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

void setGoodInitialPosition(Eigen::VectorXd& _q, XBot::ModelInterface::Ptr _model_ptr) {
    _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

}

void checkPoseEqual(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
{
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            ASSERT_NEAR(a.matrix()(i,j), b.matrix()(i,j), 1e-5);
    }
}


TEST_F(testKinematicEstimation, kinematic_floating_base_estimation)
{
    Eigen::VectorXd q, q_kinematic_estimation;
    q.setZero(_model_ptr->getJointNum());
    setGoodInitialPosition(q, _model_ptr);
    q_kinematic_estimation = q;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    _model_ptr_kinematic_estimation->setJointPosition(q_kinematic_estimation);
    _model_ptr_kinematic_estimation->update();

    std::string anchor_link = "l_sole";
    OpenSoT::floating_base_estimation::kinematic_estimation kinematic_estimation(_model_ptr_kinematic_estimation,anchor_link);
    kinematic_estimation.update();

    EXPECT_TRUE(anchor_link == kinematic_estimation.getAnchor());

    //Here anchor pose should is equal to Identity
    Eigen::Affine3d anchor_pose; anchor_pose.setIdentity();
    checkPoseEqual(anchor_pose, kinematic_estimation.getAnchorPose());
    std::cout<<"Initial anchor pose wrt world: \n"<<kinematic_estimation.getAnchorPose().matrix()<<std::endl;

    //Here floating base pose is the pose wrt the anchor
    std::string floating_base;
    _model_ptr->getFloatingBaseLink(floating_base);
    Eigen::Affine3d floating_base_pose;
    _model_ptr->getPose(floating_base, anchor_link, floating_base_pose);
    checkPoseEqual(floating_base_pose, kinematic_estimation.getFloatingBasePose());
    std::cout<<"Initial floating_base pose wrt world: \n"<<kinematic_estimation.getFloatingBasePose().matrix()<<std::endl;

    _model_ptr->setFloatingBasePose(floating_base_pose);
    _model_ptr->update();

    //Extra check for the two models
    _model_ptr->getJointPosition(q);
    _model_ptr_kinematic_estimation->getJointPosition(q_kinematic_estimation);
    EXPECT_TRUE(q == q_kinematic_estimation);
    _logger->add("q", q);
    _logger->add("q_kinematic_estimation", q_kinematic_estimation);
    std::cout<<"q initial: \n"<<q.transpose()<<std::endl;
    std::cout<<"q_kinematic_estimation initial: \n"<<q_kinematic_estimation.transpose()<<std::endl;

    //Here we create some tasks to control the floating base
    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole(new OpenSoT::tasks::velocity::Cartesian(
                                                        "l_sole", q, *_model_ptr,"l_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole(new OpenSoT::tasks::velocity::Cartesian(
                                                        "r_sole", q, *_model_ptr,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr base  (new OpenSoT::tasks::velocity::Cartesian(
                                                        "floating_base", q, *_model_ptr,floating_base, "world"));
    base->setLambda(0.1);

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims(new OpenSoT::constraints::velocity::VelocityLimits(
                                                                     M_PI, 0.001, q.size()));

    OpenSoT::AutoStack::Ptr autostack = ((l_sole + r_sole)/base/postural)<<vel_lims;


    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(),1e6));

    KDL::Frame base_goal;
    base->getReference(base_goal);
    std::cout<<"base_actual: "<<base_goal<<std::endl;
    base_goal.M.DoRotX(25.*M_PI/180.);
    base->setReference(base_goal);
    base->getReference(base_goal);
    std::cout<<"base_goal: "<<base_goal<<std::endl;
    Eigen::VectorXd dq(q.size());
    dq.setZero(dq.size());
    std::cout<<"START LOOP"<<std::endl;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr_kinematic_estimation->setJointPosition(q_kinematic_estimation);
        _model_ptr_kinematic_estimation->update();
        kinematic_estimation.update();
        _model_ptr_kinematic_estimation->getJointPosition(q_kinematic_estimation);

        for(unsigned int k = 0; k < q.size(); ++k)
            EXPECT_NEAR(q_kinematic_estimation[k], q[k], 1e-5)<<"i="<<i<<"\n q: "<<q.transpose()<<"\n"<<"q_kinematic_estimation: "<<q_kinematic_estimation.transpose()<<std::endl;
        _logger->add("q", q);
        _logger->add("q_kinematic_estimation", q_kinematic_estimation);

        Eigen::Affine3d floating_base;
        _model_ptr->getFloatingBasePose(floating_base);
        Eigen::Affine3d floating_base_kinematic_estimation;
        _model_ptr_kinematic_estimation->getFloatingBasePose(floating_base_kinematic_estimation);

//        std::cout<<"floating_base pose wrt world: \n"<<floating_base.matrix()<<std::endl;
//        std::cout<<"floating_base_kinematic_estimation pose wrt world: \n"<<floating_base_kinematic_estimation.matrix()<<std::endl;
        checkPoseEqual(floating_base, floating_base_kinematic_estimation);

        autostack->update(q);


        EXPECT_TRUE(solver->solve(dq));
        q += dq;
        q_kinematic_estimation.tail(q_kinematic_estimation.size()-6) += dq.tail(q_kinematic_estimation.size()-6);
    }

    Eigen::Affine3d actual_pose, goal_pose, actual_pose_kinematic_estimation;
    base->getReference(goal_pose);
    _model_ptr->getPose(floating_base, actual_pose);
    _model_ptr_kinematic_estimation->getPose(floating_base, actual_pose_kinematic_estimation);
    checkPoseEqual(goal_pose, actual_pose);
    checkPoseEqual(goal_pose, actual_pose_kinematic_estimation);

    std::cout<<"goal_pose: \n"<<goal_pose.matrix()<<std::endl;
    std::cout<<"actual_pose: \n"<<actual_pose.matrix()<<std::endl;
    std::cout<<"actual_pose_kinematic_estimation: \n"<<actual_pose_kinematic_estimation.matrix()<<std::endl;

    std::cout<<std::endl;

    _model_ptr_kinematic_estimation->getFloatingBasePose(floating_base_pose);
    std::string new_anchor = "r_sole";
    kinematic_estimation.setAnchor(new_anchor);
    kinematic_estimation.update();
    Eigen::Affine3d floating_base_pose_new_anchor;
    _model_ptr_kinematic_estimation->getFloatingBasePose(floating_base_pose_new_anchor);
    std::cout<<"floating_base_pose: \n"<<floating_base_pose.matrix()<<std::endl;
    std::cout<<"floating_base_pose_new_anchor: \n"<<floating_base_pose_new_anchor.matrix()<<std::endl;

    base->getReference(base_goal);
    std::cout<<"base_actual: "<<base_goal<<std::endl;
    base_goal.p[0] += 0.05;
    base->setReference(base_goal);
    base->getReference(base_goal);
    std::cout<<"base_goal: "<<base_goal<<std::endl;

    dq.setZero(dq.size());
    std::cout<<"START LOOP2"<<std::endl;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr_kinematic_estimation->setJointPosition(q_kinematic_estimation);
        _model_ptr_kinematic_estimation->update();
        kinematic_estimation.update();
        _model_ptr_kinematic_estimation->getJointPosition(q_kinematic_estimation);

        for(unsigned int k = 0; k < q.size(); ++k)
            EXPECT_NEAR(q_kinematic_estimation[k], q[k], 1e-5)<<"i="<<i<<"\n q: "<<q.transpose()<<"\n"<<"q_kinematic_estimation: "<<q_kinematic_estimation.transpose()<<std::endl;
        _logger->add("q", q);
        _logger->add("q_kinematic_estimation", q_kinematic_estimation);

        Eigen::Affine3d floating_base;
        _model_ptr->getFloatingBasePose(floating_base);
        Eigen::Affine3d floating_base_kinematic_estimation;
        _model_ptr_kinematic_estimation->getFloatingBasePose(floating_base_kinematic_estimation);

//        std::cout<<"floating_base pose wrt world: \n"<<floating_base.matrix()<<std::endl;
//        std::cout<<"floating_base_kinematic_estimation pose wrt world: \n"<<floating_base_kinematic_estimation.matrix()<<std::endl;
        checkPoseEqual(floating_base, floating_base_kinematic_estimation);

        autostack->update(q);


        EXPECT_TRUE(solver->solve(dq));
        q += dq;
        q_kinematic_estimation.tail(q_kinematic_estimation.size()-6) += dq.tail(q_kinematic_estimation.size()-6);
    }

    base->getReference(goal_pose);
    _model_ptr->getPose(floating_base, actual_pose);
    _model_ptr_kinematic_estimation->getPose(floating_base, actual_pose_kinematic_estimation);
    checkPoseEqual(goal_pose, actual_pose);
    checkPoseEqual(goal_pose, actual_pose_kinematic_estimation);

    std::cout<<"goal_pose: \n"<<goal_pose.matrix()<<std::endl;
    std::cout<<"actual_pose: \n"<<actual_pose.matrix()<<std::endl;
    std::cout<<"actual_pose_kinematic_estimation: \n"<<actual_pose_kinematic_estimation.matrix()<<std::endl;

    _logger->flush();
}

TEST_F(testKinematicEstimation, control_with_estimation)
{
    Eigen::VectorXd q;
    q.setZero(_model_ptr->getJointNum());
    setGoodInitialPosition(q, _model_ptr);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::string anchor_link = "l_sole";
    OpenSoT::floating_base_estimation::kinematic_estimation kinematic_estimation(_model_ptr, anchor_link);
    kinematic_estimation.update();
    _model_ptr->getJointPosition(q);

    std::string floating_base;
    _model_ptr->getFloatingBaseLink(floating_base);

    //Here we create some tasks to control the floating base
    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole(new OpenSoT::tasks::velocity::Cartesian(
                                                        "l_sole", q, *_model_ptr,"l_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole(new OpenSoT::tasks::velocity::Cartesian(
                                                        "r_sole", q, *_model_ptr,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr base  (new OpenSoT::tasks::velocity::Cartesian(
                                                        "floating_base", q, *_model_ptr,floating_base, "world"));
    base->setLambda(0.1);

    Eigen::Affine3d base_initial;
    base->getActualPose(base_initial);

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr vel_lims(new OpenSoT::constraints::velocity::VelocityLimits(
                                                                     M_PI, 0.001, q.size()));

    OpenSoT::AutoStack::Ptr autostack = ((l_sole + r_sole)/base/postural)<<vel_lims;


    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(autostack->getStack(), autostack->getBounds(),1e6));

    KDL::Frame base_goal;
    base->getReference(base_goal);
    std::cout<<"base_actual: "<<base_goal<<std::endl;
    base_goal.M.DoRotX(25.*M_PI/180.);
    base->setReference(base_goal);
    base->getReference(base_goal);
    std::cout<<"base_goal: "<<base_goal<<std::endl;
    Eigen::VectorXd dq(q.size());
    dq.setZero(dq.size());
    std::cout<<"START LOOP"<<std::endl;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q);
        kinematic_estimation.update();
        _model_ptr->getJointPosition(q);



        autostack->update(q);


        EXPECT_TRUE(solver->solve(dq));
        q.tail(q.size()-6) += dq.tail(q.size()-6);
    }

    Eigen::Affine3d actual_pose, goal_pose;
    base->getReference(goal_pose);
    _model_ptr->getPose(floating_base, actual_pose);

    std::cout<<"initial_pose: \n"<<base_initial.matrix()<<std::endl;
    std::cout<<"goal_pose: \n"<<goal_pose.matrix()<<std::endl;
    std::cout<<"actual_pose: \n"<<actual_pose.matrix()<<std::endl;


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
