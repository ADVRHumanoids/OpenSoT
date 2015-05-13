#include <iCub/iDynTree/yarp_kdl.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/utilities/error.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/utils/VelocityAllocation.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/SubTask.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class testQPOases_VelocityAllocation:
        public ::testing::Test
{
protected:
    std::ofstream   _log;

    testQPOases_VelocityAllocation()
    {
        _log.open("testQPOases_VelocityAllocation.py");
    }

    virtual ~testQPOases_VelocityAllocation() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};


yarp::sig::Vector getGoodInitialPosition(iDynUtils& model) {
    yarp::sig::Vector q(model.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(model.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    model.fromRobotToIDyn(leg, q, model.left_leg);
    model.fromRobotToIDyn(leg, q, model.right_leg);
    yarp::sig::Vector arm(model.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = -10.0 * M_PI/180.0;
    arm[1] = 30.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    model.fromRobotToIDyn(arm, q, model.left_arm);
    arm[1] = -arm[1];
    model.fromRobotToIDyn(arm, q, model.right_arm);
    return q;
}

void minimumVelocity(iDynUtils& model,
                     OpenSoT::DefaultHumanoidStack& DHS,
                     kinematic_chain& chain,
                     const yarp::sig::Vector& q) {
    // setting postural reference to have minimum velocity
    // on left arm, postural on the rest of the body
    yarp::sig::Vector q_postural = DHS.postural->getReference();
    for(unsigned int i = 0; i < chain.joint_numbers.size(); ++i)
        q_postural[chain.joint_numbers[i]] = q[chain.joint_numbers[i]];
    DHS.postural->setReference(q_postural);
}

//#define TRY_ON_SIMULATOR
// will try script on the simulator, without velocity allocation
//#define TRY_NVA

TEST_F(testQPOases_VelocityAllocation, tryMovingWhileKeepinTorsoStill) {

#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("hitTheBounds");
#endif

    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(model);
    yarp::sig::Vector qnva = q;
    model.updateiDyn3Model(q, true);

#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    OpenSoT::AutoStack::Ptr stack, stacknva;
    OpenSoT::DefaultHumanoidStack DHS(model, 3e-3, q);
    OpenSoT::DefaultHumanoidStack DHSnva(model, 3e-3, qnva);

    DHS.leftArm->setLambda(0.3);
    DHS.leftArm->setOrientationErrorGain(0.1);
    DHSnva.leftArm->setLambda(0.3);
    DHSnva.leftArm->setOrientationErrorGain(0.1);


    OpenSoT::SubTask::Ptr postural(
        new OpenSoT::SubTask(DHS.postural, OpenSoT::SubTask::SubTaskMap(model.torso.joint_numbers)+
                                           OpenSoT::SubTask::SubTaskMap(model.right_arm.joint_numbers)+
                                           OpenSoT::SubTask::SubTaskMap(model.left_arm.joint_numbers[2])));

    ASSERT_EQ(postural->getTaskSize(), 11);
    ASSERT_EQ(postural->getXSize(), 29);
    ASSERT_EQ(postural->getA().cols(), postural->getXSize());
    ASSERT_EQ(postural->getA().rows(), postural->getTaskSize());
    ASSERT_EQ(postural->getWeight().rows(), postural->getTaskSize());
    ASSERT_EQ(postural->getWeight().rows(), postural->getWeight().cols());
    ASSERT_EQ(postural->getb().size(), postural->getTaskSize());

    OpenSoT::SubTask::Ptr posturalnva(
        new OpenSoT::SubTask(DHSnva.postural, OpenSoT::SubTask::SubTaskMap(model.torso.joint_numbers)+
                                              OpenSoT::SubTask::SubTaskMap(model.right_arm.joint_numbers)+
                                              OpenSoT::SubTask::SubTaskMap(model.left_arm.joint_numbers[2])));

    ASSERT_EQ(posturalnva->getTaskSize(), 11);
    ASSERT_EQ(posturalnva->getXSize(), 29);
    ASSERT_EQ(posturalnva->getA().cols(), posturalnva->getXSize());
    ASSERT_EQ(posturalnva->getA().rows(), posturalnva->getTaskSize());
    ASSERT_EQ(posturalnva->getWeight().rows(), posturalnva->getTaskSize());
    ASSERT_EQ(posturalnva->getWeight().rows(), posturalnva->getWeight().cols());
    ASSERT_EQ(posturalnva->getb().size(), posturalnva->getTaskSize());

    stack =  (DHS.leftLeg + DHS.rightLeg) /
             (DHS.leftArm + DHS.rightArm) /
              postural;
    stack <<  DHS.jointLimits;

    stacknva = (DHSnva.leftLeg + DHSnva.rightLeg) /
               (DHSnva.leftArm + DHSnva.rightArm) /
                posturalnva;
    stacknva << DHSnva.jointLimits << DHSnva.velocityLimits;


    OpenSoT::VelocityAllocation(stack, 3e-3, 0.15, 0.3);
    ASSERT_DOUBLE_EQ((boost::dynamic_pointer_cast<
                        OpenSoT::constraints::velocity::VelocityLimits>(
                            postural->getConstraints().front()))->getVelocityLimits(), 0.3);
    DHSnva.velocityLimits->setVelocityLimits(0.3);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack->getStack(),
                                          stack->getBounds()));

    OpenSoT::solvers::QPOases_sot::Ptr sotnva(
        new OpenSoT::solvers::QPOases_sot(stacknva->getStack(),
                                          stacknva->getBounds()));


    //SET SOME REFERENCES
    yarp::sig::Matrix actual_pose_y = DHS.leftArm->getActualPose();
    yarp::sig::Matrix desired_pose_y = actual_pose_y;
    desired_pose_y(1,3) = actual_pose_y(1,3) + 0.1;
    desired_pose_y(2,3) = actual_pose_y(2,3) + 0.1;
    yarp::sig::Vector dq(q.size(), 0.0);
    yarp::sig::Vector dqnva(q.size(), 0.0);
    double e, enva, epost, epostnva, epost_max, epostnva_max;

    unsigned int i = 0;
    _log << "#! /usr/bin/env python" << std::endl
         << std::endl
         << "import numpy as np" << std::endl
         << "import matplotlib" << std::endl
         << "from matplotlib.pyplot import *" << std::endl;
    _log << "#t, xdot, ydot, zdot, xdotnva, ydotnva, zdotnva,"  // 1-7
         << " q0dot_torso, q1dot_torso, q2dot_torso,"           // 8-10
         << " q0dotnva_torso, q1dotnva_torso, q2dotnva_torso,"  // 11-13
         << " e, e_nva, epost, epost_nva,"                      // 14-17
         << " t_loop, t_loop_nva" << std::endl;                 // 18-19
    _log << "test_data = np.array((";

    double t_loop = 0.0;
    double t_loopnva = 0.0;
    bool settled = false;
    double settling_counter = 1.0;
    bool converged_event = false;

    DHS.leftArm->setReference(desired_pose_y);
    DHSnva.leftArm->setReference(desired_pose_y);

#ifdef TRY_ON_SIMULATOR
    double t_test = yarp::os::Time::now();
#else
    double t_test = yarp::os::SystemClock::nowSystem();
#endif
    do
    {
#ifdef TRY_ON_SIMULATOR
        double t_begin = yarp::os::Time::now();
#else
        double t_begin = yarp::os::SystemClock::nowSystem();
#endif

        model.updateiDyn3Model(q, true);
        stack->update(q);

        //minimumVelocity(model, DHS, model.left_arm, q);

        e = norm(DHS.leftArm->getb());
        epost = norm(postural->getb());
        if(epost > epost_max)
            epost_max = epost;


        EXPECT_TRUE(sot->solve(dq));
        q += dq;

#ifdef TRY_ON_SIMULATOR
#ifndef TRY_NVA
        robot.move(q);
        yarp::os::Time::delay(0.005);
#endif

        t_loop = yarp::os::Time::now() - t_begin;
#else
        t_loop = yarp::os::SystemClock::nowSystem() - t_begin;
#endif

#ifdef TRY_ON_SIMULATOR
        t_begin = yarp::os::Time::now();
#else
        t_begin = yarp::os::SystemClock::nowSystem();
#endif

        model.updateiDyn3Model(qnva, true);
        stacknva->update(qnva);

        //minimumVelocity(model, DHSnva, model.left_arm, qnva);

        enva = norm(DHSnva.leftArm->getb());
        epostnva = norm(posturalnva->getb());
        if(epostnva > epostnva_max)
            epostnva_max = epostnva;


        EXPECT_TRUE(sotnva->solve(dqnva));
        qnva+=dqnva;

#ifdef TRY_ON_SIMULATOR
#ifdef TRY_NVA
        robot.move(qnva);
        yarp::os::Time::delay(0.005);
#endif

        t_loopnva = yarp::os::Time::now() - t_begin;
#else
        t_loopnva = yarp::os::SystemClock::nowSystem() - t_begin;
#endif


#ifdef TRY_ON_SIMULATOR
        _log << "(" << yarp::os::Time::now() - t_test << ","
#else
        _log << "(" << yarp::os::SystemClock::nowSystem() - t_test << ","
#endif
            << (DHS.leftArm->getA()*dq)[0] << ","
            << (DHS.leftArm->getA()*dq)[1] << ","
            << (DHS.leftArm->getA()*dq)[2] << ","
            << (DHSnva.leftArm->getA()*dqnva)[0] << ","
            << (DHSnva.leftArm->getA()*dqnva)[1] << ","
            << (DHSnva.leftArm->getA()*dqnva)[2] << ","
            << dq[model.torso.joint_numbers[0]]  << ","
            << dq[model.torso.joint_numbers[1]]  << ","
            << dq[model.torso.joint_numbers[2]]  << ","
            << dqnva[model.torso.joint_numbers[0]]  << ","
            << dqnva[model.torso.joint_numbers[1]]  << ","
            << dqnva[model.torso.joint_numbers[2]]  << ","
            << e << "," << enva << "," << epost << "," << epostnva << ","
            << t_loop << "," << t_loopnva << ")," << std::endl;

        if(e < 1.5e-3 && enva < 1.5e-3 && !converged_event)
        {
            converged_event = true;
            std::cout << "settling";
            std::cout.flush();
        }

        if(converged_event)
        {
            settling_counter -= 3e-3;
            std::cout << ".";
            std::cout.flush();
        }

        if(settling_counter <= 0.0)
        {
            settled = true;
            std::cout << std::endl;
            std::cout.flush();
        }

    } while(e > 1.5e-3 || enva > 1.5e-3 || !settled);

    EXPECT_GT(epostnva_max, epost_max)
        << "With velocity allocation, we expect that "
        << "the maximum postural error for a certain task "
        << "will be lower than without velocity allocation.\n"
        << "In fact, having a scaled velocity for the primary "
        << "task means we are willing to invest some resources "
        << "to consistently reduce the error in the lower priority "
        << "tasks, not just when the error of the primary task is "
        << "low. Since the postural task is at a lower priority than "
        << "the cartesian tasks in this test, we want to see the peak "
        << "error for the postural to be higher without VA.";


    _log << "));" << std::endl;

    _log << "ct = figure(figsize=(8,6)); p = plot(test_data[:,0],test_data[:,(17, 18)]); title('Computation time with and without Velocity Allocation'); legend(p,('Solve time (VA)', 'Solve time (no VA)'));" << std::endl;
    _log << "vae = figure(figsize=(10.27,7.68)); subplot(3,1,1); p = plot(test_data[:,0],test_data[:,(2,3,5,6)]); title('Hand Velocity');" << std::endl;
    _log << "legend(p,('y dot (VA)', 'z dot (VA)', 'y dot (no VA)', 'z dot (no VA)'));" << std::endl;
    _log << "ylabel('Hand Velocity [m/s]'); xlabel('t [s]');" << std::endl;
    _log << "subplot(3,1,2); p = plot(test_data[:,0], np.transpose(np.vstack(((test_data[:,(7,8,9)]**2).sum(1),(test_data[:,(10,11,12)]**2).sum(1))))); title('Torso Joints Velocity');" << std::endl;
    _log << "ylabel('Torso Joint Velocity [rad/s]'); xlabel('t [s]');" << std::endl;
    _log << "legend(p,('Norm of Joint Velocity (VA)', 'Norm of Joint Velocity (no VA)'));" << std::endl;
    _log << "subplot(3,1,3); p = plot(test_data[:,0],test_data[:,(13,14,15,16)]); title('Tracking Error');" << std::endl;
    _log << "legend(p,('Left Hand tracking error (VA)','Left Hand tracking error (no VA)','Postural tracking error (VA)','Postural tracking error (no VA)'));" << std::endl;
    _log << "ct.savefig('testQPOases_VelocityAllocation_computationTime.eps', format='eps', transparent=True);" << std::endl;
    _log << "vae.savefig('testQPOases_VelocityAllocation_velocitiesAndErrors.eps',format='eps',transparent=True);" << std::endl;
    _log << "show(block=True)" << std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
