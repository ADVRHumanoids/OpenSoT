#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <gtest/gtest.h>


using namespace yarp::math;

namespace {

class testDefaultHumanoidStack: public ::testing::Test
{
protected:
    iDynUtils _robot;

    testDefaultHumanoidStack()
    {

    }

    virtual ~testDefaultHumanoidStack() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testDefaultHumanoidStack, testCreation )
{
    EXPECT_NO_THROW(OpenSoT::DefaultHumanoidStack DHS(_robot,
                                        3e-3,
                                        _robot.zeros));

    OpenSoT::DefaultHumanoidStack DHS2(_robot,
                                        3e-3,
                                        _robot.zeros);
    EXPECT_TRUE((bool)DHS2.com);
    EXPECT_TRUE((bool)DHS2.comVelocity);
    EXPECT_TRUE((bool)DHS2.convexHull);
    EXPECT_TRUE((bool)DHS2.jointLimits);
    EXPECT_TRUE((bool)DHS2.leftArm);
    EXPECT_TRUE((bool)DHS2.leftLeg);
    EXPECT_TRUE((bool)DHS2.postural);
    EXPECT_TRUE((bool)DHS2.rightArm);
    EXPECT_TRUE((bool)DHS2.rightLeg);
    EXPECT_TRUE((bool)DHS2.velocityLimits);
    EXPECT_TRUE((bool)DHS2.waist);
    EXPECT_TRUE((bool)DHS2.waist2LeftArm);
    EXPECT_TRUE((bool)DHS2.waist2RightArm);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
