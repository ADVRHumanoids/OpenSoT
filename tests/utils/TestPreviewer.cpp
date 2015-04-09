#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/Previewer.h>
#include <gtest/gtest.h>

#define dT 1.0e-2

using namespace yarp::math;

namespace OpenSoT {


/**
 * @brief The MyTrajGen class dummy trajectory generator:
 *        it creates a constant trajectory that lasts one second.
 *        The goal pose is with p=initialPose, R=I
 */
class MyTrajGen
{
    KDL::Frame f;
public:
    typedef boost::shared_ptr<MyTrajGen> Ptr;
    MyTrajGen(yarp::sig::Matrix yf) { assert(YarptoKDL(yf, f) &&
                                             "Error converting yarp::Matrix to kdl::Frame"); }
    KDL::Frame Pos(double time) { return f; }
    KDL::Twist Vel(double time) { return KDL::Twist(); }
    double Duration() { return 1.0; }
};

class testPreviewer: public ::testing::Test
{
public:
    typedef OpenSoT::Previewer<MyTrajGen> Previewer;

protected:
    iDynUtils _robot;
    OpenSoT::DefaultHumanoidStack DHS;
    Previewer::Ptr previewer;
    OpenSoT::AutoStack::Ptr autostack;

    testPreviewer() :
        _robot("coman",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        DHS(_robot,
              dT,
              _robot.zeros)
    {

        yarp::sig::Vector q = _robot.iDyn3_model.getAng();

        // defining a stack composed of size two,
        // where the task of first priority is an aggregated of leftArm and rightArm,
        // rightArm contains a convexHull constraint;
        // the task at the second priority level is an aggregated of rightLeg and leftLeg,
        // and the stack is subject to bounds jointLimits and velocityLimits

        OpenSoT::AutoStack::Ptr temp = ( (DHS.leftArm + DHS.rightArm) /
                                         (DHS.rightLeg + DHS.leftLeg) ) << DHS.jointLimits << DHS.velocityLimits;
        autostack = temp;

        MyTrajGen::Ptr trajLeftArm(new MyTrajGen(DHS.leftArm->getActualPose()));
        MyTrajGen::Ptr trajRightArm(new MyTrajGen(DHS.rightArm->getActualPose()));

        Previewer::TrajectoryBindings bindings;
        bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
        bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

        previewer.reset(new Previewer(dT, _robot, autostack, bindings));
    }

    virtual ~testPreviewer() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    bool dynamicCastWorks()
    {
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr comTask(DHS.com);
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lArmTask(DHS.leftArm);

        return
                previewer->isCoM(DHS.com) &&
                previewer->isCoM(comTask) &&
                previewer->isCoM(previewer->asCoM(comTask)) &&
                !previewer->isCartesian(DHS.com) &&
                !previewer->isCartesian(comTask) &&
                !previewer->isCartesian(previewer->asCoM(comTask)) &&
                previewer->isCartesian(DHS.leftArm) &&
                previewer->isCartesian(lArmTask) &&
                previewer->isCartesian(previewer->asCartesian(lArmTask)) &&
                !previewer->isCoM(DHS.leftArm) &&
                !previewer->isCoM(lArmTask) &&
                !previewer->isCoM(previewer->asCartesian(lArmTask));
    }

    bool cartesianPoseChangedWorks()
    {
        double threshold = 1e-2;
        yarp::sig::Vector q = _robot.iDyn3_model.getAng();
        _robot.updateiDyn3Model(q, true);
        autostack->update(q);
        previewer->reset();

        // first time we call it, it always returns true
        if(!previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged should return true "
                      << "when called first time after a reset" << std::endl;
            return false;
        }

        // but if we didn't move, it will then return false
        if(previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Matrix x = DHS.leftArm->getActualPose();
        yarp::sig::Matrix x_des = x;
        x_des(0,3) = x(0,3) + 10.0*threshold;
        DHS.leftArm->setReference(x_des);

        yarp::sig::Vector dq;
        for(unsigned int i = 0; i < 100; ++i)
        {
            _robot.updateiDyn3Model(q, true);
            autostack->update(q);
            if(previewer->solver->solve(dq))
                q+=dq;
            else
                std::cout << "--- Error: unable to solve stack" << std::endl;
        }

        if(yarp::math::norm(DHS.leftArm->getActualPose().getCol(3) - x.getCol(3)) < threshold)
        {
            std::cout << "--- Error following desired reference:" << std::endl;
            std::cout << "--- actual pose is: " << DHS.leftArm->getActualPose().getCol(3).toString() << std::endl
                      << "--- desired pose is: " << DHS.leftArm->getReference().getCol(3).toString() << std::endl;
            return false; // why didn't we move?
        }

        // but if we didn't move, it will then return false
        if(!previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged is returning false while "
                      << "the cartesian pose changed" << std::endl;
            return false;
        }


        return true;
    }

    bool jointSpaceConfigurationChangedWorks()
    {
        yarp::sig::Vector zero = _robot.iDyn3_model.getAng(); zero.zero();
        _robot.updateiDyn3Model(zero);
        previewer->reset();

        if(previewer->jointSpaceConfigurationChanged(1e2))
            return false;   //by default, qNode is 0

        double threshold = 1e-3;

        do
        {
            previewer->q = tests_utils::getRandomAngles(
                _robot.iDyn3_model.getJointBoundMin(),
                _robot.iDyn3_model.getJointBoundMax(),
                _robot.iDyn3_model.getNrOfDOFs());
        } while(yarp::math::norm(previewer->q - zero) <= threshold);


        if(!previewer->jointSpaceConfigurationChanged(threshold))
        {
            std::cout << "Joint space configuration changed, but "
                      << "jointSpaceConfigurationChanged returns false" << std::endl;
            return false;
        }


        return true;
    }

    bool shouldCheckSelfCollisionWorks()
    {
        return true;
    }

};

TEST_F(testPreviewer, testDynamicCast)
{
    ASSERT_TRUE(dynamicCastWorks());
}

TEST_F(testPreviewer, checkResults)
{
    Previewer::Results results;
    if(!previewer->check(0.1,3,&results))
    {
        std::cout << "Trajectory unfeasible!";

        std::cout << "Logged " << results.failures.size() << " failures:" << std::endl;
        for(unsigned int i = 0; i < results.failures.size(); ++i)
        {
            std::cout << "@t:" << results.failures[i].t
                      << " - " << Previewer::Results::reasonToString(results.failures[i].reason)
                      << std::endl;
        }
    }
    else
    {
        std::cout << "Logged " << results.trajectory.size() << " trajectory nodes:" << std::endl;
        for(unsigned int i = 0; i < results.trajectory.size(); ++i)
        {
            std::cout << "@t:" << results.trajectory[i].t
                      << " - " << results.trajectory[i].q.toString()
                      << std::endl;
        }
    }
    ASSERT_TRUE(true);
}

TEST_F(testPreviewer, testShouldCheckSelfCollision)
{
    ASSERT_TRUE(cartesianPoseChangedWorks());

    ASSERT_TRUE(jointSpaceConfigurationChangedWorks());

    ASSERT_TRUE(shouldCheckSelfCollisionWorks());
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
