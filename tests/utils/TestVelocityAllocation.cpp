#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/VelocityAllocation.h>
#include <gtest/gtest.h>


using namespace yarp::math;

namespace {

class testVelocityAllocation: public ::testing::Test
{
protected:
    iDynUtils _robot;
    OpenSoT::DefaultHumanoidStack DHS, DHS2;

    testVelocityAllocation() :
        _robot("coman",
               std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
               std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        DHS(_robot, 3e-3, _robot.zeros),
        DHS2(_robot, 3e-3, _robot.zeros)
    {

    }

    virtual ~testVelocityAllocation() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testVelocityAllocation, testConstructors )
{
    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftArm and rightArm,
    // rightArm contains a convexHull constraint;
    // the task at the second priority level is an aggregated of rightLeg and leftLeg,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack =
        (DHS.leftArm + DHS.rightArm)
        / (DHS.rightLeg + DHS.leftLeg)
        / (DHS.com << DHS.comVelocity)
        / DHS.postural;
    autoStack << DHS.jointLimits;

    OpenSoT::AutoStack::Ptr autoStack2 =
        (DHS2.leftArm + DHS2.rightArm)
        / (DHS2.rightLeg + DHS2.leftLeg)
        / (DHS2.com << DHS2.comVelocity)
        / DHS2.postural;
    autoStack2 << DHS2.jointLimits;

    OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack stack = autoStack2->getStack();

    OpenSoT::VelocityAllocation(autoStack,
                                0.1,
                                0.3);

    OpenSoT::VelocityAllocation(stack,
                                0.1,
                                0.3);

    for(unsigned int i = 0; i < stack.size(); ++i)
    {
        if(i == 0)
            ASSERT_DOUBLE_EQ(0.1+0.2*1,0.1);
        else if(i == 1)
            ASSERT_DOUBLE_EQ(0.1+0.2*1,0.1666666666666666666666666667);
        else if(i == 2)
            ASSERT_DOUBLE_EQ(0.1+0.2*1,0.2333333333333333333333333334);
        else if(i == 3)
            ASSERT_DOUBLE_EQ(0.1+0.2*1,0.3);
    }

    for(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task :
        autoStack->getStack())
    {
        EXPECT_DOUBLE_EQ(
        std::cout << "Task "<< i << " has velocity bounds" <<
            boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    task->getConstraints().front()
                         )->getVelocityLimits(),0.1+i*0.2/3.0)

        ++i;
    }

    for(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task :
        stack)
    {
        EXPECT_DOUBLE_EQ(
        std::cout << "Task "<< i << " has velocity bounds" <<
            boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    task->getConstraints().front()
                         )->getVelocityLimits(),0.1+i*0.2/3.0)

        ++i;
    }


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
