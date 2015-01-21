#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class testQPOases_SubTask: public ::testing::Test
{
protected:
    std::ofstream _log;

    testQPOases_SubTask()
    {
        _log.open("testQPOases_SubTask.m");
    }

    virtual ~testQPOases_SubTask() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
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
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

TEST_F(testQPOases_SubTask, testSolveUsingSubTasks)
{
    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q, dq;
    q = model.iDyn3_model.getAng();
    OpenSoT::DefaultHumanoidStack DHS(model, 1e-3, q);

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS.leftArm_Orientation / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(subTaskTest->getStack(), subTaskTest->getBounds()));
    ASSERT_EQ(DHS.leftArm_Orientation->getTaskSize(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().cols(),q.size());
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().cols(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getb().size(),3);

    OpenSoT::tasks::Aggregated::TaskPtr task = DHS.leftArm_Orientation;

    ASSERT_EQ(task->getTaskSize(),3);
    ASSERT_EQ(task->getA().rows(),3);
    ASSERT_EQ(task->getA().cols(),q.size());
    ASSERT_EQ(task->getWeight().rows(),3);
    ASSERT_EQ(task->getWeight().cols(),3);
    ASSERT_EQ(task->getb().size(),3);
    ASSERT_EQ(task->getTaskSize(),3);

    for(unsigned int i = 0; i < 1000; ++i) {
        yarp::sig::Matrix H = task->getA().transposed() * task->getWeight() * task->getA();
        yarp::sig::Vector g = -1.0 * task->getLambda() * task->getA().transposed() * task->getWeight() * task->getb();

        ASSERT_EQ(H.rows(),q.size());
        ASSERT_EQ(H.cols(),q.size());
        ASSERT_EQ(g.size(),q.size());
    }

    ASSERT_TRUE(solver->solve(dq));

    q += .01;
    model.updateiDyn3Model(q,true);
    subTaskTest->update(q);
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveUsingSubTasksAndAggregated)
{
    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q, dq;
    q = model.iDyn3_model.getAng();
    OpenSoT::DefaultHumanoidStack DHS(model, 1e-3, q);

    OpenSoT::AutoStack::Ptr subTaskTest = ((DHS.leftArm_Orientation + DHS.rightArm) / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(subTaskTest->getStack(), subTaskTest->getBounds()));

    ASSERT_TRUE(solver->solve(dq));

    q += .01;
    model.updateiDyn3Model(q,true);
    subTaskTest->update(q);
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveCartesianThroughSubTasksAndAggregated)
{
    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q, dq;
    q = getGoodInitialPosition(model);
    model.updateiDyn3Model(q,true);
    OpenSoT::DefaultHumanoidStack DHS(model, 1e-3, q);

    OpenSoT::AutoStack::Ptr subTaskTest = ((DHS.leftArm_Position + DHS.leftArm_Orientation) / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(subTaskTest->getStack(), subTaskTest->getBounds()));
    yarp::sig::Matrix actualPose = DHS.leftArm->getActualPose();
    yarp::sig::Matrix desiredPose = actualPose; desiredPose(0,3) = actualPose(0,3)+0.1;

    unsigned int iterations = 10000;
    while(yarp::math::norm(DHS.leftArm->getb()) > 1e-4 && iterations > 0)
    {
        model.updateiDyn3Model(q, true);
        subTaskTest->update(q);
        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    ASSERT_TRUE(yarp::math::norm(DHS.leftArm->getb()) <= 1e-4);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
