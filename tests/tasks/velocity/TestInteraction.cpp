#include <idynutils/tests_utils.h>
#include <idynutils/cartesian_utils.h>
#include <idynutils/idynutils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>

using namespace yarp::math;
using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

namespace {

class testInteractionTask: public ::testing::Test
{
protected:
    testInteractionTask()
    {

    }

    virtual ~testInteractionTask() {

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

/**
 * @brief computeWallForce computes the wrench performed by a spring attached to the wall and the robot ee
 * @param xEE the current position of the end effector of the robot
 * @param xWall the rest position of the wall
 * @param Kw the cartesian stiffness in the end effector frame of reference
 * @return
 */
yarp::sig::Vector computeWallForce(yarp::sig::Matrix& xEE, yarp::sig::Matrix& xWall, const yarp::sig::Matrix& Kw)
{
    yarp::sig::Vector pe, oe;
    cartesian_utils::computeCartesianError(xEE, xWall, pe, oe);

    return Kw * yarp::math::cat(pe, oe);
}

TEST_F(testInteractionTask, testInteractionTask_wrench)
{
    std::string urdf_file = std::string(OPENSOT_TESTS_ROBOTS_DIR) + "bigman/bigman.urdf";
    std::string srdf_file = std::string(OPENSOT_TESTS_ROBOTS_DIR) + "bigman/bigman.srdf";

    iDynUtils _robot("bigman", urdf_file, srdf_file);

    yarp::sig::Vector q = getGoodInitialPosition(_robot);
    _robot.updateiDyn3Model(q, true);

    std::string distal_link = "l_wrist";
    std::string ft_sensor_link = "l_arm_ft";
    std::string base_link = "torso";

    _robot.switchAnchor(base_link);

    // We consider the robot in contact with the wall, with 0 contact force:
    yarp::sig::Matrix xWall = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(base_link),
                                                             _robot.iDyn3_model.getLinkIndex(distal_link));
    KDL::Frame SensorTDistal_KDL = _robot.iDyn3_model.getPositionKDL(_robot.iDyn3_model.getLinkIndex(ft_sensor_link),
                                                                     _robot.iDyn3_model.getLinkIndex(distal_link));
    // The robot has a certain Compliance:

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) = 1E-4;
        C(i+3, i+3) = 1E-1;
    }
    // We compute the contact force as:
    yarp::sig::Vector distal_WallWrench = computeWallForce(xWall, xWall, yarp::math::pinv(C));
    // We have to report the wrench in sensor frame
    KDL::Wrench distal_WallWrench_KDL; cartesian_utils::fromYarpVectortoKDLWrench(distal_WallWrench, distal_WallWrench_KDL);
    KDL::Wrench sensor_WallWrench_KDL = SensorTDistal_KDL*distal_WallWrench_KDL;
    yarp::sig::Vector sensor_WallWrench; cartesian_utils::fromKDLWrenchtoYarpVector(sensor_WallWrench_KDL, sensor_WallWrench);

    // And we set as a feedback
    for(unsigned int i = 0; i < 4; ++i)
        _robot.iDyn3_model.setSensorMeasurement(i, -1.0*sensor_WallWrench);

    ASSERT_EQ(yarp::math::norm(sensor_WallWrench),0.0);

    Interaction::Ptr interactionTask(new Interaction("interaction::l_wrist", q, _robot,
                                                     distal_link,
                                                     base_link,
                                                     ft_sensor_link));

    interactionTask->setCompliance(C);

    EXPECT_TRUE(C == interactionTask->getCompliance());

    int T = 1000;
    VelocityLimits::Ptr joint_velocity_limits(new VelocityLimits(0.3, (double)(1.0/T), q.size()));

    JointLimits::Ptr joint_limits(new JointLimits(q,
                                                _robot.iDyn3_model.getJointBoundMax(),
                                                _robot.iDyn3_model.getJointBoundMin()));

    //Create the SoT
    std::vector< OpenSoT::Task<Matrix, Vector>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(interactionTask);

    std::list< OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);


    std::cout<<distal_link<<" actual position in world@t0: "<<std::endl;
    cartesian_utils::printHomogeneousTransform(interactionTask->getActualPose());
    std::cout <<"Applied wrench@t0: ["<<interactionTask->getActualWrench().toString()<<"]"<<std::endl;
    ASSERT_EQ(yarp::math::norm(interactionTask->getActualWrench()),0.0);

    yarp::sig::Vector base_WallWrenchDesired = interactionTask->getActualWrench();
    base_WallWrenchDesired[0] += 20.0;
    base_WallWrenchDesired[1] += 5.0;
    base_WallWrenchDesired[2] += 5.0;
//    base_WallWrenchDesired[3] += 5.0;
//    base_WallWrenchDesired[4] += 1.0;
//    base_WallWrenchDesired[5] += 5.0;
    interactionTask->setReferenceWrench(base_WallWrenchDesired);

    yarp::sig::Vector dq(q.size(), 0.0);
    unsigned int max_iterations = 50000;
    for(unsigned int i = 0; i < max_iterations; ++i)
    {
        _robot.updateiDyn3Model(q, true);
        SensorTDistal_KDL = _robot.iDyn3_model.getPositionKDL(_robot.iDyn3_model.getLinkIndex(ft_sensor_link),
                                                              _robot.iDyn3_model.getLinkIndex(distal_link));

        yarp::sig::Matrix xEE = interactionTask->getActualPose();
        std::cout<<"\n\n---@t"<<double(i)/T<<"------------\n";
        std::cout<<distal_link<<" actual position in world: "<<std::endl;
        cartesian_utils::printHomogeneousTransform(xEE);
        distal_WallWrench = computeWallForce(xEE, xWall, pinv(C));
        // We have to transform the wrench in sensor frame
        cartesian_utils::fromYarpVectortoKDLWrench(distal_WallWrench, distal_WallWrench_KDL);
        sensor_WallWrench_KDL = SensorTDistal_KDL*distal_WallWrench_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(sensor_WallWrench_KDL, sensor_WallWrench);
        std::cout<<"Wall Wrench in sensor frame: ["<<sensor_WallWrench.toString()<<"]"<<std::endl;

         for(unsigned int i = 0; i < _robot.iDyn3_model.getNrOfFTSensors(); ++i)
             _robot.iDyn3_model.setSensorMeasurement(i, -1.0*sensor_WallWrench);

         interactionTask->update(q);
         joint_constraints->update(q);

         std::cout <<"Applied wrench: ["<<interactionTask->getActualWrench().toString()<<"]"<<std::endl;
         std::cout<<"Force error: ["<<interactionTask->forceError.toString()<<"]"<<std::endl;
         std::cout<<"Torque error: ["<<interactionTask->torqueError.toString()<<"]"<<std::endl;

         sot.solve(dq);
         q += dq;
    }

    EXPECT_LT(norm(interactionTask->forceError),1e-3);
    EXPECT_LT(norm(interactionTask->torqueError),1e-3);

/*    _robot.updateiDyn3Model(q, true);
    yarp::sig::Vector expected = C * base_WallWrenchDesired;
    yarp::sig::Vector pe, oe;
    yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(ft_sensor_link));
    cartesian_utils::computeCartesianError(x, xWall, pe, oe);
    yarp::sig::Vector solution = yarp::math::cat(pe, oe);

    std::cout<<"EXPECTED:   "<<expected.toString()<<std::endl;
    std::cout<<"ACTUAL:     "<<solution.toString()<<std::endl;*/
}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
