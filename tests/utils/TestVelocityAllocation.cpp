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
    OpenSoT::DefaultHumanoidStack _DHS;

    testVelocityAllocation() :
        _robot("coman",
               std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
               std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        _DHS(_robot, 3e-3, _robot.zeros)
    {

    }

    virtual ~testVelocityAllocation() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testVelocityAllocation, testAllocation )
{
    const double minimum_velocity = 0.1;
    const double maximum_velocity = 0.3;
    const int stack_size = 4;

    /**
     * testing the algorithm for a 4 tasks stack,
     * min_velocity=0.1, max_velocity=0.3
     **/
    for(unsigned int i = 0; i < stack_size; ++i)
    {
        if(i == 0)
            ASSERT_DOUBLE_EQ(minimum_velocity+i*(maximum_velocity - minimum_velocity)/(stack_size-1),
                             0.1);
        else if(i == 1)
            ASSERT_DOUBLE_EQ(minimum_velocity+i*(maximum_velocity - minimum_velocity)/(stack_size-1),
                             0.1666666666666666666666666667);
        else if(i == 2)
            ASSERT_DOUBLE_EQ(minimum_velocity+i*(maximum_velocity - minimum_velocity)/(stack_size-1),
                             0.2333333333333333333333333334);
        else if(i == 3)
            ASSERT_DOUBLE_EQ(minimum_velocity+i*(maximum_velocity - minimum_velocity)/(stack_size-1),
                             0.3);
    }
}

TEST_F(testVelocityAllocation, testConstructorStack)
{
    // testing constructor applies velocity bounds correctly when no velocity bounds existing
    {
        // defining a stack of size four,
        // where the task of first priority is an aggregated of leftArm and rightArm,
        // rightArm contains a convexHull constraint;
        // the task at the second priority level is an aggregated of rightLeg and leftLeg,
        // and the stack is subject to bounds jointLimits and velocityLimits
        OpenSoT::AutoStack::Ptr autoStack =
            (_DHS.leftArm + _DHS.rightArm)
            / (_DHS.rightLeg + _DHS.leftLeg)
            / _DHS.com
            / _DHS.postural;
        autoStack << _DHS.jointLimits;

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack stack = autoStack->getStack();


        const double minimum_velocity = 0.1;
        const double maximum_velocity = 0.3;

        OpenSoT::VelocityAllocation(autoStack,
                                    3e-3,
                                    minimum_velocity,
                                    maximum_velocity);


        unsigned int i = 0;
        for(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task :
            autoStack->getStack())
        {
            ASSERT_EQ(task->getConstraints().size(),1);
            EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    task->getConstraints().front()
                            )->getVelocityLimits(),
                minimum_velocity+i*(maximum_velocity - minimum_velocity)/(autoStack->getStack().size()-1));
            if(i == 0)
                EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                                    OpenSoT::constraints::velocity::VelocityLimits>(
                                        task->getConstraints().front()
                                                )->getVelocityLimits(),
                                 0.1);
            else if(i == 1)
                EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                                    OpenSoT::constraints::velocity::VelocityLimits>(
                                        task->getConstraints().front()
                                                )->getVelocityLimits(),
                                 0.16666666666666666);
            else if(i == 2)
                EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                                    OpenSoT::constraints::velocity::VelocityLimits>(
                                        task->getConstraints().front()
                                                )->getVelocityLimits(),
                                 0.23333333333333334);
            else if(i == 3)
                EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                                    OpenSoT::constraints::velocity::VelocityLimits>(
                                        task->getConstraints().front()
                                                )->getVelocityLimits(),
                                 0.3);
            ++i;
        }
    }

    // testing constructor applies velocity bounds correctly velocity bounds exist on tasks
    {
        OpenSoT::DefaultHumanoidStack DHS(_robot, 3e-3, _robot.zeros);

        OpenSoT::AutoStack::Ptr autoStack =
            (DHS.leftArm + DHS.rightArm)
            / (DHS.rightLeg + DHS.leftLeg)
            / (DHS.com << DHS.comVelocity)
            / DHS.postural;
        autoStack << DHS.jointLimits;

        const double high_velocity = 0.8;

        DHS.postural->getConstraints().push_back(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr(
            new OpenSoT::constraints::velocity::VelocityLimits(high_velocity,
                                                               3e-3,
                                                               DHS.postural->getXSize())));

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack stack = autoStack->getStack();
        ASSERT_EQ(autoStack->getStack()[3]->getConstraints().size(),1);
        ASSERT_TRUE((bool)boost::dynamic_pointer_cast<
                        OpenSoT::constraints::velocity::VelocityLimits>(
                            autoStack->getStack()[3]->getConstraints().front()));
        ASSERT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        autoStack->getStack()[3]->getConstraints().front())->getVelocityLimits(),high_velocity);

        const double minimum_velocity = 0.1;
        const double maximum_velocity = 0.3;

        OpenSoT::VelocityAllocation(autoStack,
                                    3e-3,
                                    minimum_velocity,
                                    maximum_velocity);

        ASSERT_EQ(autoStack->getStack()[3]->getConstraints().size(),1);
        ASSERT_TRUE((bool)boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        autoStack->getStack()[3]->getConstraints().front()));
        ASSERT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        autoStack->getStack()[3]->getConstraints().front())->getVelocityLimits(),maximum_velocity);

    }
}

TEST_F(testVelocityAllocation, testConstructorAutoStack)
{
    // testing constructor applies velocity bounds correctly when no velocity bounds exist on tasks,
    // and no velocity bounds existing on stack
    {
        // defining a stack of size four,
        // where the task of first priority is an aggregated of leftArm and rightArm,
        // rightArm contains a convexHull constraint;
        // the task at the second priority level is an aggregated of rightLeg and leftLeg,
        // and the stack is subject to bounds jointLimits and velocityLimits
        OpenSoT::AutoStack::Ptr autoStack =
            (_DHS.leftArm + _DHS.rightArm)
            / (_DHS.rightLeg + _DHS.leftLeg)
            / _DHS.com
            / _DHS.postural;
        autoStack << _DHS.jointLimits;

        const double minimum_velocity = 0.1;
        const double maximum_velocity = 0.3;

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack stack = autoStack->getStack();

        OpenSoT::VelocityAllocation(stack,
                                    3e-3,
                                    minimum_velocity,
                                    maximum_velocity);



        unsigned int i = 0;
        for(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task :
            stack)
        {
            ASSERT_EQ(task->getConstraints().size(),1) << "Task "
                                                       << task->getTaskID()
                                                       << " should have 1 constraint";
            EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    task->getConstraints().front()
                        )->getVelocityLimits(),
                minimum_velocity+i*(maximum_velocity - minimum_velocity)/(stack.size()-1));

            ++i;
        }
    }

    // testing constructor applies velocity bounds correctly velocity bounds exist on tasks,
    // and no velocity bounds exist on stack
    {
        OpenSoT::DefaultHumanoidStack DHS(_robot, 3e-3, _robot.zeros);

        OpenSoT::AutoStack::Ptr autoStack =
            (DHS.leftArm + DHS.rightArm)
            / (DHS.rightLeg + DHS.leftLeg)
            / (DHS.com << DHS.comVelocity)
            / DHS.postural;
        autoStack << DHS.jointLimits;

        const double high_velocity = 0.8;

        DHS.postural->getConstraints().push_back(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr(
            new OpenSoT::constraints::velocity::VelocityLimits(high_velocity,
                                                               3e-3,
                                                               DHS.postural->getXSize())));

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::Stack stack = autoStack->getStack();
        ASSERT_EQ(autoStack->getStack()[3]->getConstraints().size(),1);
        ASSERT_TRUE((bool)boost::dynamic_pointer_cast<
                        OpenSoT::constraints::velocity::VelocityLimits>(
                            autoStack->getStack()[3]->getConstraints().front()));
        ASSERT_EQ(boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        autoStack->getStack()[3]->getConstraints().front())->getVelocityLimits(),high_velocity);

        const double minimum_velocity = 0.1;
        const double maximum_velocity = 0.3;

        OpenSoT::VelocityAllocation(autoStack,
                                    3e-3,
                                    minimum_velocity,
                                    maximum_velocity);

        ASSERT_EQ(autoStack->getStack()[3]->getConstraints().size(),1);
        ASSERT_TRUE((bool)boost::dynamic_pointer_cast<
                    OpenSoT::constraints::velocity::VelocityLimits>(
                        autoStack->getStack()[3]->getConstraints().front()));
        ASSERT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                autoStack->getStack()[3]->getConstraints().front())->getVelocityLimits(),
                        maximum_velocity);
    }

    // testing constructor applies velocity bounds correctly when no velocity bounds exist on tasks,
    // and velocity bounds exist on stack
    {
        OpenSoT::DefaultHumanoidStack DHS(_robot, 3e-3, _robot.zeros);

        OpenSoT::AutoStack::Ptr autoStack =
            (DHS.leftArm + DHS.rightArm)
            / (DHS.rightLeg + DHS.leftLeg)
            / (DHS.com << DHS.comVelocity)
            / DHS.postural;

        ASSERT_EQ(autoStack->getBoundsList().size(),0);
        const double high_velocity = 0.8;
        OpenSoT::constraints::velocity::VelocityLimits::Ptr velocityLimits(
            new OpenSoT::constraints::velocity::VelocityLimits(high_velocity,
                                                               3e-3,
                                                               DHS.postural->getXSize()));
        autoStack << velocityLimits;
        ASSERT_EQ(autoStack->getBoundsList().size(),1);

        const double minimum_velocity = 0.1;
        const double maximum_velocity = 0.3;

        OpenSoT::VelocityAllocation(autoStack,
                                    3e-3,
                                    minimum_velocity,
                                    maximum_velocity);

        ASSERT_EQ(autoStack->getBoundsList().size(),1);
        ASSERT_DOUBLE_EQ(boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                autoStack->getBoundsList().front())->getVelocityLimits(),
                         maximum_velocity);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
