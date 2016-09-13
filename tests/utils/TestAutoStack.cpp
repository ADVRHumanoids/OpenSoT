#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <gtest/gtest.h>


using namespace yarp::math;

namespace {

class testAutoStack: public ::testing::Test
{
protected:
    iDynUtils _robot;
    OpenSoT::DefaultHumanoidStack DHS;

    testAutoStack() :
        _robot("coman",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        DHS(_robot,
              3e-3,
              _robot.zeros)
    {

    }

    virtual ~testAutoStack() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testAutoStack, test_getOperationalSpaceTask_with_task_id)
{
    using namespace OpenSoT;
    std::string task_id = "CoM";

    AutoStack::Ptr auto_stack = (DHS.right2LeftLeg)/
            (DHS.com + DHS.leftArm)/
            DHS.postural;
    auto_stack->update(yarp::sig::Vector(_robot.iDyn3_model.getNrOfDOFs(),0.));

    OpenSoT::solvers::QPOases_sot::TaskPtr com_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(com_task != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::CoM> task_CoM =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(com_task);
    EXPECT_TRUE(task_CoM != NULL);
    EXPECT_TRUE(task_CoM->getTaskID().compare(task_id) == 0);

    task_id = "mammeta";
    OpenSoT::solvers::QPOases_sot::TaskPtr mammeta_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(mammeta_task == NULL);

    task_id = "cartesian::l_wrist";
    OpenSoT::solvers::QPOases_sot::TaskPtr Cartesian_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(Cartesian_task != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> left_arm =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task);
    EXPECT_TRUE(left_arm != NULL);
    EXPECT_TRUE(left_arm->getTaskID().compare(task_id) == 0);

    task_id = "cartesian:r2l_sole";
    OpenSoT::solvers::QPOases_sot::TaskPtr Cartesian_task2 = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(Cartesian_task2 != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> right_left_leg =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task2);
    EXPECT_TRUE(right_left_leg != NULL);
    EXPECT_TRUE(right_left_leg->getTaskID().compare(task_id) == 0);
}

TEST_F(testAutoStack, test_getOperationalSpaceTask_with_links)
{
    using namespace OpenSoT;
    std::string base_link = "world";
    std::string distal_link = "CoM";

    AutoStack::Ptr auto_stack = (DHS.right2LeftLeg)/
            (DHS.com + DHS.leftArm)/
            DHS.postural;
    auto_stack->update(yarp::sig::Vector(_robot.iDyn3_model.getNrOfDOFs(),0.));

    OpenSoT::solvers::QPOases_sot::TaskPtr com_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(com_task != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::CoM> task_CoM =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(com_task);
    EXPECT_TRUE(task_CoM != NULL);
    EXPECT_TRUE(task_CoM->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(task_CoM->getDistalLink().compare(distal_link) == 0);

    base_link = "mammeta";
    OpenSoT::solvers::QPOases_sot::TaskPtr mammeta_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(mammeta_task == NULL);

    base_link = "world";
    distal_link = _robot.left_arm.end_effector_name;
    OpenSoT::solvers::QPOases_sot::TaskPtr Cartesian_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(Cartesian_task != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> left_arm =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task);
    EXPECT_TRUE(left_arm != NULL);
    EXPECT_TRUE(left_arm->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(left_arm->getDistalLink().compare(distal_link) == 0);

    base_link = _robot.left_leg.end_effector_name;
    distal_link = _robot.right_leg.end_effector_name;
    OpenSoT::solvers::QPOases_sot::TaskPtr Cartesian_task2 = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(Cartesian_task2 != NULL);
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> right_left_leg =
            boost::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task2);
    EXPECT_TRUE(right_left_leg != NULL);
    EXPECT_TRUE(right_left_leg->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(right_left_leg->getDistalLink().compare(distal_link) == 0);
}

TEST_F(testAutoStack, testOperatorPlus )
{
    using namespace OpenSoT;
    tasks::Aggregated::Ptr aggr1 = DHS.leftArm + DHS.rightArm;
    EXPECT_EQ(aggr1->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr2 = DHS.leftLeg + DHS.rightLeg;
    EXPECT_EQ(aggr2->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr3 = aggr1 + aggr2;
    EXPECT_EQ(aggr3->getTaskList().size(), 4);

    aggr1->setLambda(.1);
    aggr2->setLambda(.2);

    tasks::Aggregated::Ptr aggr4 = aggr1 + aggr2;
    EXPECT_EQ(aggr4->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr5 = DHS.leftArm + DHS.rightArm
                                 + DHS.leftLeg + DHS.rightLeg;
    EXPECT_EQ(aggr5->getTaskList().size(), 4);

    tasks::Aggregated::Ptr aggr6 = aggr1 + DHS.leftLeg;
    EXPECT_EQ(aggr6->getTaskList().size(), 3);

    tasks::Aggregated::Ptr aggr7 = DHS.leftLeg + aggr1;
    EXPECT_EQ(aggr7->getTaskList().size(), 3);

    /*
     * checking constraints are carried over in the right way
     */
    tasks::Aggregated::Ptr aggr8 = DHS.leftLeg + DHS.rightLeg;
    aggr8->getConstraints().push_back(DHS.convexHull);

    tasks::Aggregated::Ptr aggr9 = DHS.leftLeg + DHS.rightLeg;
    aggr9->getConstraints().push_back(DHS.comVelocity);

    tasks::Aggregated::Ptr aggr10 = DHS.leftLeg + DHS.rightLeg;
    aggr10->getConstraints().push_back(DHS.velocityLimits);
    aggr10->getConstraints().push_back(DHS.comVelocity);

    EXPECT_EQ(aggr8->getConstraints().size(),1);
    EXPECT_EQ(aggr9->getConstraints().size(),1);
    EXPECT_EQ(aggr10->getConstraints().size(),2);
    EXPECT_EQ((aggr8+aggr9)->getConstraints().size(),2);
    EXPECT_EQ((aggr8+aggr10)->getConstraints().size(),3);
    EXPECT_EQ((aggr9+aggr10)->getConstraints().size(),2);
}

TEST_F(testAutoStack, testOperatorFraction)
{
    using namespace OpenSoT;

    tasks::Aggregated::Ptr aggr1 = DHS.leftArm + DHS.rightArm;
    EXPECT_TRUE(aggr1->getTaskList().size() == 2);

    AutoStack::Ptr auto1 = DHS.leftArm / DHS.rightArm;
    EXPECT_TRUE(auto1->getStack().size() == 2);

    AutoStack::Ptr auto2(new AutoStack(solvers::QPOases_sot::Stack(1,aggr1)));
    EXPECT_EQ(auto2->getStack().size(), 1);

    AutoStack::Ptr auto3 = auto2 / DHS.leftLeg;
    ASSERT_EQ(auto3->getStack().size(), 2);
    EXPECT_TRUE(auto3->getStack()[1] == DHS.leftLeg);

    AutoStack::Ptr auto4 = DHS.leftLeg / auto2;
    ASSERT_EQ(auto4->getStack().size(), 2);
    EXPECT_TRUE(auto4->getStack()[0] == DHS.leftLeg);

    AutoStack::Ptr auto5 = auto3 / auto4;
    ASSERT_TRUE(auto5->getStack().size() == 4);
    EXPECT_TRUE(auto5->getStack()[0] == aggr1);
    EXPECT_TRUE(auto5->getStack()[1] == DHS.leftLeg);
    EXPECT_TRUE(auto5->getStack()[2] == DHS.leftLeg);
    EXPECT_TRUE(auto5->getStack()[3] == aggr1);
}

TEST_F(testAutoStack, testOperatorRedirection)
{
    using namespace OpenSoT;

    tasks::Aggregated::Ptr aggr1 = DHS.leftArm + DHS.rightArm;
    tasks::Aggregated::TaskPtr leftArm = DHS.leftArm;
    AutoStack::Ptr auto1 = DHS.leftArm / DHS.rightArm;

    tasks::Aggregated::Ptr aggr1Copy = aggr1 << DHS.velocityLimits;
    EXPECT_TRUE(aggr1->getConstraints().size() == 1);
    ASSERT_TRUE(aggr1->getConstraints().front() == DHS.velocityLimits);
    EXPECT_TRUE(aggr1Copy->getConstraints().size() == 1);
    ASSERT_TRUE(aggr1Copy->getConstraints().front() == DHS.velocityLimits);

    tasks::Aggregated::TaskPtr leftArmCopy = leftArm << DHS.velocityLimits;
    EXPECT_TRUE(leftArm->getConstraints().size() == 1);
    ASSERT_TRUE(leftArm->getConstraints().front() == DHS.velocityLimits);
    EXPECT_TRUE(leftArmCopy->getConstraints().size() == 1);
    ASSERT_TRUE(leftArmCopy->getConstraints().front() == DHS.velocityLimits);

    aggr1 << DHS.jointLimits;
    ASSERT_TRUE(aggr1->getConstraints().size() == 2);
    ASSERT_TRUE(aggr1->getConstraints().back() == DHS.jointLimits);

    leftArm << DHS.jointLimits;
    EXPECT_TRUE(leftArm->getConstraints().size() == 2);
    ASSERT_TRUE(leftArm->getConstraints().back() == DHS.jointLimits);

    auto1 << DHS.velocityLimits;
    EXPECT_TRUE(auto1->getBoundsList().size() == 1);

    auto1 << DHS.convexHull << DHS.comVelocity;
    EXPECT_TRUE(auto1->getBoundsList().size() == 3);

    AutoStack::Ptr auto2 =
    (leftArm + (DHS.rightArm << DHS.convexHull))
        / (DHS.rightLeg + DHS.leftLeg) << DHS.jointLimits << DHS.velocityLimits;
    EXPECT_TRUE(auto2->getBoundsList().size() == 2);
    EXPECT_TRUE(auto2->getStack().size() == 2);
    EXPECT_TRUE(boost::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().size() == 2);
    EXPECT_EQ(boost::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().front()->getConstraints().size(), 2);
    EXPECT_EQ(boost::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().back()->getConstraints().size(), 1);
    EXPECT_EQ(boost::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().size(), 2);
    EXPECT_EQ(boost::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().front()->getConstraints().size(), 0);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
