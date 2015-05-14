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
 *        it creates a trajectory that lasts t seconds, where the position
 *        of the frame gets interpolated from the starting position to the
 *        end position. The rotation remains constant.
 *        The goal pose is with p=finalPose, R=initialR
 */
class MyTrajGen
{
    KDL::Frame b;
    KDL::Frame f;
    double t;
public:
    typedef boost::shared_ptr<MyTrajGen> Ptr;
    MyTrajGen(yarp::sig::Matrix yb,
              yarp::sig::Matrix yf, double t) : t(t) { bool res = YarptoKDL(yb, b) &&
                                             YarptoKDL(yf, f); assert( res &&
                                             "Error converting yarp::Matrix to kdl::Frame"); }
    KDL::Frame Pos(double time) {
        KDL::Frame ref = b;
        if(time > t)
            time = t;

        ref.p = (t-time)/t*b.p + time/t*f.p;
        return ref;
    }
    KDL::Twist Vel(double time) { return KDL::Twist(); }
    double Duration() { return t; }
};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 20.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

class testPreviewer: public ::testing::Test
{
public:
    typedef OpenSoT::Previewer<MyTrajGen> Previewer;

protected:
    iDynUtils _robot;
    yarp::sig::Vector q;
    OpenSoT::DefaultHumanoidStack DHS;
    Previewer::Ptr previewer;
    OpenSoT::AutoStack::Ptr autostack;

    testPreviewer() :
        _robot("coman",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        q(getGoodInitialPosition(_robot)),
        DHS((_robot.updateiDyn3Model(q, true),_robot),
              dT,
              q)
    {
        // defining a stack composed of size two,
        // where the task of first priority is an aggregated of leftArm and rightArm,
        // rightArm contains a convexHull constraint;
        // the task at the second priority level is an aggregated of rightLeg and leftLeg,
        // and the stack is subject to bounds jointLimits and velocityLimits

        OpenSoT::AutoStack::Ptr temp = ( (DHS.leftArm + DHS.rightArm) /
                                         (DHS.rightLeg + DHS.leftLeg) ) << DHS.jointLimits << DHS.velocityLimits;
        autostack = temp;
        autostack->update(q);

        /* by default, we have constant references */
        MyTrajGen::Ptr trajLeftArm(new MyTrajGen(DHS.leftArm->getActualPose(),
                                                 DHS.leftArm->getActualPose(),
                                                 1.0));
        MyTrajGen::Ptr trajRightArm(new MyTrajGen(DHS.rightArm->getActualPose(),
                                                  DHS.rightArm->getActualPose(),
                                                  1.0));

        Previewer::TrajectoryBindings bindings;
        bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
        bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

        previewer.reset(new Previewer(dT, _robot, autostack, bindings));
    }

    virtual ~testPreviewer() {

    }

    virtual void SetUp() {
        DHS.leftArm->setOrientationErrorGain(0.1);
        DHS.rightArm->setOrientationErrorGain(0.1);
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
        q = _robot.iDyn3_model.getAng();
        _robot.updateiDyn3Model(q, true);
        autostack->update(q);
        previewer->resetPreviewer();

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
        previewer->resetPreviewer();

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

        double threshold = 1e-2;
        q = _robot.iDyn3_model.getAng();
        _robot.updateiDyn3Model(q, true);
        autostack->update(q);
        previewer->resetPreviewer();

        // first time we call it, it always returns true
        if(!previewer->shouldCheckSelfCollision(threshold))
        {
            std::cout << "--- shouldCheckSelfCollision() should return true "
                      << "when called first time after a reset" << std::endl;
            return false;
        } else std::cout << "+++ First call to shouldCheckSelfCollision "
                         << "succesfully returns true" << std::endl;

        // but if we didn't move, it will then return false
        if(previewer->shouldCheckSelfCollision(threshold))
        {
            std::cout << "--- shouldCheckSelfCollision is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Matrix x = DHS.leftArm->getActualPose();
        yarp::sig::Matrix x_des = x;
        x_des(0,3) = x(0,3) + 10.0*threshold;
        DHS.leftArm->setReference(x_des);
        DHS.rightArm->setReference(DHS.rightArm->getActualPose());

        DHS.leftArm->setOrientationErrorGain(0.0);
        DHS.rightArm->setOrientationErrorGain(0.0);
        yarp::sig::Vector dq;
        for(unsigned int i = 0; i < 100; ++i)
        {
            if(previewer->solver->solve(dq))
            {
                q+=dq;

                _robot.updateiDyn3Model(q, true);
                autostack->update(q);

                double dXL = yarp::math::norm(DHS.leftArm->getActualPose().getCol(3) - x.getCol(3));
                double dXR = yarp::math::norm(DHS.rightArm->getActualPose().getCol(3) - DHS.rightArm->getReference().getCol(3));
                std::cout << "dXL: " << dXL << std::endl;
                std::cout << "dXR: " << dXR << std::endl;
                if( dXL >= threshold || dXR >= threshold)
                {
                    x = DHS.leftArm->getActualPose();
                    if(!previewer->shouldCheckSelfCollision(threshold))
                    {
                        std::cout << "--- Error! cartesian reference changed, but "
                                  << "shouldCheckSelfCollision did not return true" << std::endl;
                        return false;
                    }
                } else {
                    if(previewer->shouldCheckSelfCollision(threshold))
                    {
                        std::cout << "--- Error! cartesian reference did not change much, but "
                                  << "shouldCheckSelfCollision did return true" << std::endl;
                        return false;
                    }
                }
            }
            else
                std::cout << "--- Error: unable to solve stack" << std::endl;
        }

        // we didn't move, shouldCheckSelfCollision should return false
        if(previewer->shouldCheckSelfCollision(threshold))
        {
            std::cout << "--- shouldCheckSelfCollision is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Vector previous_q = q;
        do
        {
            q = tests_utils::getRandomAngles(
                _robot.iDyn3_model.getJointBoundMin(),
                _robot.iDyn3_model.getJointBoundMax(),
                _robot.iDyn3_model.getNrOfDOFs());
        } while(yarp::math::norm(previous_q - q) <= threshold);
        _robot.updateiDyn3Model(q,true);
        autostack->update(q);

        if(!previewer->shouldCheckSelfCollision())
        {
            std::cout << "Joint space configuration changed, but "
                      << "shouldCheckSelfCollision returns false" << std::endl;
            return false;
        }


        return true;
    }

};

TEST_F(testPreviewer, testDynamicCast)
{
    ASSERT_TRUE(dynamicCastWorks());
}

TEST_F(testPreviewer, checkStaticConvergence)
{
    ASSERT_FALSE(_robot.checkSelfCollision());

    Previewer::Results results;
    bool preview_success = previewer->check(0.1,3,&results);
    EXPECT_FALSE(preview_success);
    EXPECT_EQ(results.failures.size(),0);

    preview_success = previewer->check(1.0,3,&results);
    EXPECT_TRUE(preview_success);

    if(!preview_success)
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
        std::cout << "\nLogged " << results.trajectory.size() << " trajectory nodes:" << std::endl;
        for(unsigned int i = 0; i < results.trajectory.size(); ++i)
        {
            std::cout << "@t:" << results.trajectory[i].t
                      << " - " << results.trajectory[i].q.toString()
                      << std::endl;
        }
    }
}

TEST_F(testPreviewer, checkFeasibleConvergence)
{
    /* feasible trajectory: 10 cm in 10secs */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = 10.0;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm, 1e-2, 1e-3));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm, 1e-2, 1e-3));

    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);
    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_TRUE(eL < 1e-3);
    EXPECT_TRUE(eR < 1e-3);

    EXPECT_TRUE(preview_success);
}

TEST_F(testPreviewer, checkAutoConvergenceCheck)
{
    /* feasible trajectory: 10 cm in 10secs */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double duration = 10.0;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    Previewer::Results results;
    bool preview_success = previewer->check(std::numeric_limits<double>::infinity(),
                                            3,&results);
    ASSERT_TRUE(preview_success);
    EXPECT_TRUE(results.trajectory.back().t < 2.0*duration);
}

TEST_F(testPreviewer, checkUnfeasibleConvergence)
{
    /* feasible trajectory: 1 cm in .2sec */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = .2;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm,.1));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm,.1));

    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    ASSERT_FALSE(preview_success);
    EXPECT_EQ(results.failures.size(), 0);
}

TEST_F(testPreviewer, checkUnfeasibleBoundedness)
{
    /* feasible trajectory: 1 cm in .2sec */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = .2;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm, 5.e-2));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm, 5.e-2));

    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(10.0*duration,3,&results);

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));

    EXPECT_TRUE(eL < 1e-2);
    EXPECT_TRUE(eR < 1e-2);

    ASSERT_FALSE(preview_success);

    bool error_unbounded = false;
    for(unsigned int i = 0; i < results.failures.size(); ++i)
    {
        if(results.failures[i].reason == Previewer::Results::ERROR_UNBOUNDED)
            error_unbounded = true;
    }
    EXPECT_TRUE(error_unbounded);
}

TEST_F(testPreviewer, checkSelfCollision)
{
    /* feasible trajectory with collision on l_arm: 10 cm in .1sec */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(1,3) = 0.;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb;
    double duration = 10;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);
    EXPECT_FALSE(preview_success);

    bool self_collision = false;
    for(unsigned int i = 0; i < results.failures.size(); ++i)
    {
        if(results.failures[i].reason == Previewer::Results::COLLISION_EVENT)
            self_collision = true;
    }
    EXPECT_TRUE(self_collision);
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
