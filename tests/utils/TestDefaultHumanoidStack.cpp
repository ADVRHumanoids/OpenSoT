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

    testDefaultHumanoidStack() : _robot("coman",
                                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
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
                                        cartesian_utils::toEigen(_robot.zeros)));

    OpenSoT::DefaultHumanoidStack DHS2(_robot,
                                        3e-3,
                                        cartesian_utils::toEigen(_robot.zeros));
    EXPECT_TRUE((bool)DHS2.com);
    EXPECT_TRUE((bool)DHS2.com_XY);
    EXPECT_TRUE((bool)DHS2.com_Z);

    EXPECT_TRUE((bool)DHS2.comVelocity);
    EXPECT_TRUE((bool)DHS2.convexHull);
    EXPECT_TRUE((bool)DHS2.jointLimits);

    EXPECT_TRUE((bool)DHS2.leftArm);
    EXPECT_TRUE((bool)DHS2.leftArm_Position);
    EXPECT_TRUE((bool)DHS2.leftArm_Orientation);

    EXPECT_TRUE((bool)DHS2.leftLeg);
    EXPECT_TRUE((bool)DHS2.leftLeg_Position);
    EXPECT_TRUE((bool)DHS2.leftLeg_Orientation);

    EXPECT_TRUE((bool)DHS2.postural);

    EXPECT_TRUE((bool)DHS2.rightArm);
    EXPECT_TRUE((bool)DHS2.rightArm_Position);
    EXPECT_TRUE((bool)DHS2.rightArm_Orientation);

    EXPECT_TRUE((bool)DHS2.rightLeg);
    EXPECT_TRUE((bool)DHS2.rightLeg_Position);
    EXPECT_TRUE((bool)DHS2.rightLeg_Orientation);

    EXPECT_TRUE((bool)DHS2.velocityLimits);

    EXPECT_TRUE((bool)DHS2.waist);
    EXPECT_TRUE((bool)DHS2.waist_Position);
    EXPECT_TRUE((bool)DHS2.waist_Position_XY);
    EXPECT_TRUE((bool)DHS2.waist_Position_Z);
    EXPECT_TRUE((bool)DHS2.waist_Orientation);

    EXPECT_TRUE((bool)DHS2.waist2LeftArm);
    EXPECT_TRUE((bool)DHS2.waist2LeftArm_Position);
    EXPECT_TRUE((bool)DHS2.waist2LeftArm_Orientation);

    EXPECT_TRUE((bool)DHS2.waist2RightArm);
    EXPECT_TRUE((bool)DHS2.waist2RightArm_Position);
    EXPECT_TRUE((bool)DHS2.waist2RightArm_Orientation);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
