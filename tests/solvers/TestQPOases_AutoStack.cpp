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
    OpenSoT::DefaultHumanoidStack DHS(model, 1e-3, cartesian_utils::toEigen(q));

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS.leftArm / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::QPOases_sot::Ptr solver(
        new OpenSoT::solvers::QPOases_sot(subTaskTest->getStack(), subTaskTest->getBounds()));
    yarp::sig::Matrix actualPose = cartesian_utils::fromEigentoYarp(DHS.leftArm->getActualPose());
    yarp::sig::Matrix desiredPose = actualPose; desiredPose(0,3) = actualPose(0,3)+0.1;

    unsigned int iterations = 10000;
    while(sqrt(DHS.leftArm->getb().squaredNorm()) > 1e-4 && iterations > 0)
    {
        double oldBoundsNorm = sqrt(DHS.jointLimits->getbLowerBound().squaredNorm());
        model.updateiDyn3Model(q, true);
        subTaskTest->update(model.getAng());

        if(yarp::math::norm(dq) > 1e-3)
            EXPECT_NE(oldBoundsNorm,sqrt(DHS.jointLimits->getbLowerBound().squaredNorm()));

        Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
        ASSERT_TRUE(solver->solve(_dq));
        dq = cartesian_utils::fromEigentoYarp(_dq);
        q += dq;
    }

    ASSERT_TRUE(sqrt(DHS.leftArm->getb().squaredNorm()) <= 1e-4);
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
                                                       "6d::r_leg",
                                                       cartesian_utils::toEigen(q),model,"r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::l_arm",
                                                       cartesian_utils::toEigen(q),model,"LSoftHand", "world"));
    yarp::sig::Matrix l_arm_ref = cartesian_utils::fromEigentoYarp(l_arm->getActualPose());
    l_arm_ref(2,3) += 0.1;
    l_arm->setReference(cartesian_utils::toEigen(l_arm_ref));

    OpenSoT::tasks::velocity::Cartesian::Ptr r_arm(new OpenSoT::tasks::velocity::Cartesian(
                                                       "6d::r_arm",
                                                       cartesian_utils::toEigen(q),model,"RSoftHand", "world"));
    yarp::sig::Matrix r_arm_ref = cartesian_utils::fromEigentoYarp(r_arm->getActualPose());
    r_arm_ref(2,3) += 0.1;
    r_arm->setReference(cartesian_utils::toEigen(r_arm_ref));

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(
                                                         cartesian_utils::toEigen(q)));

    Eigen::VectorXd joint_bound_max = model.getJointBoundMax();
    Eigen::VectorXd joint_bound_min = model.getJointBoundMin();
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds(
        new OpenSoT::constraints::velocity::JointLimits(
                    cartesian_utils::toEigen(q), joint_bound_max, joint_bound_min));

    double sot_speed_limit = 0.5;
    double dT = 0.001;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new OpenSoT::constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));

    OpenSoT::constraints::velocity::ConvexHull::Ptr convex_hull_constraint(
                new OpenSoT::constraints::velocity::ConvexHull(
                    cartesian_utils::toEigen(q), model, 0.02));

    OpenSoT::AutoStack::Ptr AutoStack = (((r_leg)<<convex_hull_constraint)/
                                         ((l_arm+r_arm)<<convex_hull_constraint)/
                                         ((postural)<<convex_hull_constraint));
    AutoStack->getBoundsList().push_back(joint_bounds);
    AutoStack->getBoundsList().push_back(velocity_bounds);
    /* the following is not needed, as getBounds() from line #156 will compute generateAll()
       and correctly recompute the bounds. Notice that, if the q from line #155 was instead
       different from that of line #134, the update(q_new) would have been necessary */
    // AutoStack->getBounds()->update(q);

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
        AutoStack->update(cartesian_utils::toEigen(q));

        Eigen::VectorXd _dq(q.size()); _dq.setZero(q.size());
        ASSERT_TRUE(solver->solve(_dq));
        dq = cartesian_utils::fromEigentoYarp(_dq);
        q += dq;
    }

    EXPECT_TRUE(sqrt(l_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"l_arm getb norm "<<sqrt(l_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_arm->getb().squaredNorm()) <= 1e-4);
    std::cout<<"r_arm getb norm "<<sqrt(r_arm->getb().squaredNorm())<<std::endl;
    EXPECT_TRUE(sqrt(r_leg->getb().squaredNorm()) <= 1e-4);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
