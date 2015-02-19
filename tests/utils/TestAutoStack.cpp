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
    EXPECT_TRUE(boost::reinterpret_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().size() == 2);
    EXPECT_EQ(boost::reinterpret_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().front()->getConstraints().size(), 2);
    EXPECT_EQ(boost::reinterpret_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().back()->getConstraints().size(), 1);
    EXPECT_EQ(boost::reinterpret_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().size(), 2);
    EXPECT_EQ(boost::reinterpret_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().front()->getConstraints().size(), 0);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
