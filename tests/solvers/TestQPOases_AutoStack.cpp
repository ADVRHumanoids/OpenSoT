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

class testQPOases_AutoStack: public ::testing::Test
{
protected:
    std::ofstream _log;

    testQPOases_AutoStack()
    {
        _log.open("testQPOases_AutoStack.m");
    }

    virtual ~testQPOases_AutoStack() {
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

TEST_F(testQPOases_AutoStack, testSolveUsingAutoStack)
{
    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q, dq;
    q = getGoodInitialPosition(model);
    dq = q; q.zero();
    model.updateiDyn3Model(q,true);
    OpenSoT::DefaultHumanoidStack DHS(model, 1e-3, q);

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS.leftArm / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(subTaskTest->getStack(), subTaskTest->getBounds()));
    yarp::sig::Matrix actualPose = DHS.leftArm->getActualPose();
    yarp::sig::Matrix desiredPose = actualPose; desiredPose(0,3) = actualPose(0,3)+0.1;

    unsigned int iterations = 10000;
    while(yarp::math::norm(DHS.leftArm->getb()) > 1e-4 && iterations > 0)
    {
        double oldBoundsNorm = yarp::math::norm(DHS.jointLimits->getbLowerBound());
        model.updateiDyn3Model(q, true);
        subTaskTest->update(model.iDyn3_model.getAng());

        if(yarp::math::norm(dq) > 1e-3)
            EXPECT_NE(oldBoundsNorm,
                      yarp::math::norm(DHS.jointLimits->getbLowerBound()));

        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    ASSERT_TRUE(yarp::math::norm(DHS.leftArm->getb()) <= 1e-4);
}

TEST_F(testQPOases_AutoStack, testComplexAutoStack)
{
    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q, dq;
    q = getGoodInitialPosition(model);
    dq = q; dq.zero();
    model.updateiDyn3Model(q,true);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_leg(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_leg",q,model,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_arm",q,model,"LSoftHand", "world"));
    yarp::sig::Matrix l_arm_ref = l_arm->getActualPose();
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(l_arm_ref);

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_arm",q,model,"RSoftHand", "world"));
    yarp::sig::Matrix r_arm_ref = r_arm->getActualPose();
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(r_arm_ref);

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));

    yarp::sig::Vector joint_bound_max = model.iDyn3_model.getJointBoundMax();
    yarp::sig::Vector joint_bound_min = model.iDyn3_model.getJointBoundMin();
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds(
        new OpenSoT::constraints::velocity::JointLimits(q,joint_bound_max,joint_bound_min));

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new OpenSoT::constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint(
                new OpenSoT::constraints::velocity::ConvexHull(q, model, 0.02));

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    AutoStack->getBounds()->update(q);

    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(AutoStack->getStack(), AutoStack->getBounds()));

    typedef OpenSoT::solvers::QPOases_sot::Stack::iterator it_stack;
    for(it_stack i0 = AutoStack->getStack().begin();
        i0 != AutoStack->getStack().end(); ++i0)
    {
        ASSERT_EQ((*i0)->getConstraints().size(),1);
    }

    unsigned int iterations = 5000;
    for(unsigned int i = 0; i < iterations; ++i)
    {
        model.updateiDyn3Model(q, true);
        AutoStack->update(q);

        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    EXPECT_TRUE(yarp::math::norm(l_arm->getb()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<yarp::math::norm(l_arm->getb())<<std::endl;
    EXPECT_TRUE(yarp::math::norm(r_arm->getb()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<yarp::math::norm(r_arm->getb())<<std::endl;
    EXPECT_TRUE(yarp::math::norm(r_leg->getb()) <= 1e-4);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
