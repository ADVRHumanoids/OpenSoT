#include <gtest/gtest.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/AutoStack.h>
#include "DefaultHumanoidStack.h"
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <fstream>
#include "../common.h"


#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class testQPOases_AutoStack: public TestBase
{
protected:
    std::ofstream _log;

    testQPOases_AutoStack(): TestBase("coman_floating_base")
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
    Eigen::VectorXd _q = _model_ptr.getNeutralQ();
    _q[_model_ptr.getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr.getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr.getQIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("LShYaw")] = -10.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr.getQIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("RShYaw")] = 10.0*M_PI/180.0;
    _q[_model_ptr.getQIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}


TEST_F(testQPOases_AutoStack, testSolveUsingAutoStack)
{
    XBot::ModelInterface::Ptr model = _model_ptr;

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq.setZero(model->getNv());

    model->setJointPosition(q);
    model->update();

    OpenSoT::DefaultHumanoidStack::Ptr DHS;
    DHS.reset(new OpenSoT::DefaultHumanoidStack(*model,
          3e-3,
          "Waist",
          "LSoftHand", "RSoftHand",
          "l_sole", "r_sole", 0.3));

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

        subTaskTest->update(Eigen::VectorXd(0));

        if(dq.norm() > 1e-3)
            EXPECT_NE(oldBoundsNorm,sqrt(DHS->jointLimits->getbLowerBound().squaredNorm()));

        ASSERT_TRUE(solver->solve(dq));
        q = _model_ptr->sum(q, dq);
    }

    ASSERT_TRUE(sqrt(DHS->leftArm->getb().squaredNorm()) <= 1e-4);
}

TEST_F(testQPOases_AutoStack, testComplexAutoStack)
{
    XBot::ModelInterface::Ptr model = _model_ptr;

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq.setZero(model->getNv());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_leg",
                                                       *model,"r_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::l_arm",
                                                       *model,"LSoftHand", "world");
    Eigen::MatrixXd l_arm_ref = l_arm->getActualPose();
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(l_arm_ref);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_arm",
                                                       *model,"RSoftHand", "world");
    Eigen::MatrixXd r_arm_ref = r_arm->getActualPose();
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(r_arm_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(*model);

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds =
        std::make_shared<OpenSoT::constraints::velocity::JointLimits>(
                    *model, joint_bound_max, joint_bound_min);

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds =
        std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*model,
            sot_speed_limit,
            dT);


    std::list<std::string> _links_in_contact;
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint =
                std::make_shared<OpenSoT::constraints::velocity::ConvexHull>(*model, _links_in_contact, 0.02);

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    /* the following is not needed, as getBounds() from line #156 will compute generateAll()
       and correctly recompute the bounds. Notice that, if the q from line #155 was instead
       different from that of line #134, the update(q_new) would have been necessary */
    // AutoStack->getBounds()->update(q);

    OpenSoT::solvers::iHQP::Ptr solver =
        std::make_shared<OpenSoT::solvers::iHQP>(AutoStack->getStack(), AutoStack->getBounds(), 1e6);


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
        AutoStack->update(Eigen::VectorXd(0));

        ASSERT_TRUE(solver->solve(dq));
        q = model->sum(q, dq);
    }

    EXPECT_TRUE(sqrt(l_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<sqrt(l_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<sqrt(r_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_leg->getb().squaredNorm()) <= 1e-4);

}

TEST_F(testQPOases_AutoStack, testAutoStackConstructor)
{
    XBot::ModelInterface::Ptr model = _model_ptr;

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq.setZero(model->getNv());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_leg",
                                                       *model,"r_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::l_arm",
                                                       *model,"LSoftHand", "world");
    Eigen::MatrixXd l_arm_ref = l_arm->getActualPose();
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(l_arm_ref);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_arm",
                                                       *model,"RSoftHand", "world");
    Eigen::MatrixXd r_arm_ref = r_arm->getActualPose();
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(r_arm_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(*model);

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds =
        std::make_shared<OpenSoT::constraints::velocity::JointLimits>(
                    *model, joint_bound_max, joint_bound_min);

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds =
        std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*model,
            sot_speed_limit,
            dT);


    std::list<std::string> _links_in_contact;
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint =
                std::make_shared<OpenSoT::constraints::velocity::ConvexHull>(*model, _links_in_contact, 0.02);

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    /* the following is not needed, as getBounds() from line #156 will compute generateAll()
       and correctly recompute the bounds. Notice that, if the q from line #155 was instead
       different from that of line #134, the update(q_new) would have been necessary */
    // AutoStack->getBounds()->update(q);

    OpenSoT::solvers::iHQP::Ptr solver =
        std::make_shared<OpenSoT::solvers::iHQP>(*AutoStack, 1e6);


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
        AutoStack->update(Eigen::VectorXd(0));

        ASSERT_TRUE(solver->solve(dq));
        q = model->sum(q, dq);
    }

    EXPECT_TRUE(sqrt(l_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<sqrt(l_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<sqrt(r_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_leg->getb().squaredNorm()) <= 1e-4);

}

TEST_F(testQPOases_AutoStack, testAutoStacks)
{
    XBot::ModelInterface::Ptr model = _model_ptr;

    Eigen::VectorXd q, dq;
    q = getGoodInitialPosition(*model);
    dq.setZero(model->getNv());

    model->setJointPosition(q);
    model->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg= std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_leg",
                                                       *model,"r_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_leg= std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::l_leg",
                                                       *model,"l_sole", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm= std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::l_arm",
                                                       *model,"LSoftHand", "world");

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm= std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
                                                       "6d::r_arm",
                                                       *model,"RSoftHand", "world");

    OpenSoT::tasks::velocity::Postural::Ptr postural= std::make_shared<OpenSoT::tasks::velocity::Postural>(*model);

    Eigen::VectorXd joint_bound_max, joint_bound_min;
    model->getJointLimits(joint_bound_min, joint_bound_max);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds
        = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(
                    *model, joint_bound_max, joint_bound_min);

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds= std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(
            *model, sot_speed_limit,
            dT);


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
