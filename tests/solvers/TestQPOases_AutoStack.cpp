#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <fstream>


#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class testQPOases_AutoStack: public ::testing::Test
{
protected:
    std::ofstream _log;

    testQPOases_AutoStack()
    {
        _log.open("testQPOases_AutoStack.m");
    }

    virtual ~testQPOases_AutoStack() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface& _model_ptr) {
    Eigen::VectorXd _q(_model_ptr.getJointNum());
    _q.setZero();
    _q[_model_ptr.getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr.getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr.getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("LShYaw")] = -10.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr.getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("RShYaw")] = 10.0*M_PI/180.0;
    _q[_model_ptr.getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

TEST_F(testQPOases_AutoStack, testSolveUsingAutoStack)
{
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq = q; dq.setZero(dq.size());

    model->setJointPosition(q);
    model->update();

    OpenSoT::DefaultHumanoidStack::Ptr DHS;
    DHS.reset(new OpenSoT::DefaultHumanoidStack(*model,
          3e-3,
          "Waist",
          "LSoftHand", "RSoftHand",
          "l_sole", "r_sole", 0.3,
          q));

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS->leftArm / DHS->postural)
                                            << DHS->jointLimits << DHS->velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds(),1e6));



    Eigen::MatrixXd actualPose = DHS->leftArm->getActualPose();
    Eigen::MatrixXd desiredPose = actualPose;
    desiredPose(0,3) = actualPose(0,3)+0.1;

    unsigned int iterations = 10000;
    while(sqrt(DHS->leftArm->getb().squaredNorm()) > 1e-4 && iterations > 0)
    {
        double oldBoundsNorm = sqrt(DHS->jointLimits->getbLowerBound().squaredNorm());

        model->setJointPosition(q);
        model->update();

        subTaskTest->update(q);

        if(dq.norm() > 1e-3)
            EXPECT_NE(oldBoundsNorm,sqrt(DHS->jointLimits->getbLowerBound().squaredNorm()));

        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    ASSERT_TRUE(sqrt(DHS->leftArm->getb().squaredNorm()) <= 1e-4);
}

TEST_F(testQPOases_AutoStack, testComplexAutoStack)
{
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq = q; dq.setZero(dq.size());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_leg",
                                                       q,*model,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_arm",
                                                       q,*model,"LSoftHand", "world"));
    Eigen::MatrixXd l_arm_ref = l_arm->getActualPose();
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(l_arm_ref);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_arm",
                                                       q,*model,"RSoftHand", "world"));
    Eigen::MatrixXd r_arm_ref = r_arm->getActualPose();
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(r_arm_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds(
        new OpenSoT::constraints::velocity::JointLimits(
                    q, joint_bound_max, joint_bound_min));

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new OpenSoT::constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));


    std::list<std::string> _links_in_contact;
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint(
                new OpenSoT::constraints::velocity::ConvexHull(q, *model, _links_in_contact, 0.02));

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    /* the following is not needed, as getBounds() from line #156 will compute generateAll()
       and correctly recompute the bounds. Notice that, if the q from line #155 was instead
       different from that of line #134, the update(q_new) would have been necessary */
    // AutoStack->getBounds()->update(q);

    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(AutoStack->getStack(), AutoStack->getBounds(), 1e6));


    typedef OpenSoT::solvers::iHQP::Stack::iterator it_stack;
    for(it_stack i0 = AutoStack->getStack().begin();
        i0 != AutoStack->getStack().end(); ++i0)
    {
        ASSERT_EQ((*i0)->getConstraints().size(),1);
    }

    unsigned int iterations = 5000;
    for(unsigned int i = 0; i < iterations; ++i)
    {
        model->setJointPosition(q);
        model->update();
        AutoStack->update(q);

        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    EXPECT_TRUE(sqrt(l_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<sqrt(l_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<sqrt(r_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_leg->getb().squaredNorm()) <= 1e-4);

}

TEST_F(testQPOases_AutoStack, testAutoStackConstructor)
{
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq = q; dq.setZero(dq.size());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_leg",
                                                       q,*model,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_arm",
                                                       q,*model,"LSoftHand", "world"));
    Eigen::MatrixXd l_arm_ref = l_arm->getActualPose();
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(l_arm_ref);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_arm",
                                                       q,*model,"RSoftHand", "world"));
    Eigen::MatrixXd r_arm_ref = r_arm->getActualPose();
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(r_arm_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds(
        new OpenSoT::constraints::velocity::JointLimits(
                    q, joint_bound_max, joint_bound_min));

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new OpenSoT::constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));


    std::list<std::string> _links_in_contact;
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint(
                new OpenSoT::constraints::velocity::ConvexHull(q, *model, _links_in_contact, 0.02));

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    /* the following is not needed, as getBounds() from line #156 will compute generateAll()
       and correctly recompute the bounds. Notice that, if the q from line #155 was instead
       different from that of line #134, the update(q_new) would have been necessary */
    // AutoStack->getBounds()->update(q);

    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(*AutoStack, 1e6));


    typedef OpenSoT::solvers::iHQP::Stack::iterator it_stack;
    for(it_stack i0 = AutoStack->getStack().begin();
        i0 != AutoStack->getStack().end(); ++i0)
    {
        ASSERT_EQ((*i0)->getConstraints().size(),1);
    }

    unsigned int iterations = 5000;
    for(unsigned int i = 0; i < iterations; ++i)
    {
        model->setJointPosition(q);
        model->update();
        AutoStack->update(q);

        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    EXPECT_TRUE(sqrt(l_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<sqrt(l_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<sqrt(r_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_leg->getb().squaredNorm()) <= 1e-4);

}

TEST_F(testQPOases_AutoStack, testAutoStacks)
{
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq = q; dq.setZero(dq.size());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_leg",
                                                       q,*model,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_leg(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_leg",
                                                       q,*model,"l_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_arm",
                                                       q,*model,"LSoftHand", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_arm",
                                                       q,*model,"RSoftHand", "world"));

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds(
        new OpenSoT::constraints::velocity::JointLimits(
                    q, joint_bound_max, joint_bound_min));

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new OpenSoT::constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));


    OpenSoT::AutoStack::Ptr AutoStackA = ((l_leg + r_leg)/(l_arm + r_arm))<<joint_bounds;

    OpenSoT::AutoStack::Ptr AutoStackB = AutoStackA;
    AutoStackB /= postural;
    AutoStackB<<velocity_bounds;

    EXPECT_EQ(AutoStackA->getStack().size(), 2);
    EXPECT_EQ(AutoStackB->getStack().size(), 3);

    EXPECT_EQ(AutoStackA->getBoundsList().size(), 1);
    EXPECT_EQ(AutoStackB->getBoundsList().size(), 2);


}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
