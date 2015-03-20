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
#include <OpenSoT/tasks/velocity/all.h>
#include <fstream>

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
 * @brief computeWallForce computes the reaction wrench performed by a spring attached to the wall and the robot ee
 * @param xEE the current position of the end effector of the robot
 * @param xWall the rest position of the wall
 * @param Kw the cartesian stiffness in the end effector frame of reference
 * @return
 */
yarp::sig::Vector computeWallForce(yarp::sig::Matrix& xWall, yarp::sig::Matrix& xEE, const yarp::sig::Matrix& Kw)
{
    KDL::Frame xWall_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(xWall, xWall_KDL);
    KDL::Frame xEE_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(xEE, xEE_KDL);

    yarp::sig::Vector pe(3,0.0), oe(3,0.0);
    pe[0] = -xEE_KDL.p.x() + xWall_KDL.p.x();
    pe[1] = -xEE_KDL.p.y() + xWall_KDL.p.y();
    pe[2] = -xEE_KDL.p.z() + xWall_KDL.p.z();

    double xEERoll, xEEPitch, xEEYaw;
    xEE_KDL.M.GetRPY(xEERoll, xEEPitch, xEEYaw);
    double xWallRoll, xWallPitch, xWallYaw;
    xWall_KDL.M.GetRPY(xWallRoll, xWallPitch, xWallYaw);

    oe[0] = -xEERoll + xWallRoll;
    oe[1] = -xEEPitch + xWallPitch;
    oe[2] = -xEEYaw + xWallYaw;

    return Kw * yarp::math::cat(pe, oe);
}

TEST_F(testInteractionTask, testComputeWallForce)
{
    yarp::sig::Matrix TW(4,4); TW = TW.eye();
    yarp::sig::Matrix TEE(4,4); TEE = TW;

    TEE(0,3) = 2.0;
    TEE(1,3) = 3.0;
    TEE(2,3) = -4.0;

    yarp::sig::Matrix Kw(6,6); Kw = Kw.eye();
    Kw(0,0) = 10.0; Kw(1,1) = 10.0; Kw(2,2) = 10.0;

    yarp::sig::Vector reactionWrench = computeWallForce(TW, TEE, Kw);

    std::cout<<"Reaction Wrench from Wall: ["<<reactionWrench.toString()<<"]"<<std::endl;

    EXPECT_DOUBLE_EQ(reactionWrench[0], -Kw(0,0)*TEE(0,3));
    EXPECT_DOUBLE_EQ(reactionWrench[1], -Kw(1,1)*TEE(1,3));
    EXPECT_DOUBLE_EQ(reactionWrench[2], -Kw(2,2)*TEE(2,3));

    TEE = TEE.eye();
    KDL::Frame TEE_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(TEE, TEE_KDL);
    TEE_KDL.M.DoRotX(M_PI_2);
    cartesian_utils::fromKDLFrameToYARPMatrix(TEE_KDL, TEE);

    reactionWrench = computeWallForce(TW, TEE, Kw);

    std::cout<<"Reaction Wrench from Wall: ["<<reactionWrench.toString()<<"]"<<std::endl;
    EXPECT_DOUBLE_EQ(reactionWrench[3], -Kw(3,3)*M_PI_2);

    TEE = TEE.eye();
    TEE_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(TEE, TEE_KDL);
    TEE_KDL.M.DoRotY(-M_PI_2/2.0);
    cartesian_utils::fromKDLFrameToYARPMatrix(TEE_KDL, TEE);

    reactionWrench = computeWallForce(TW, TEE, Kw);

    std::cout<<"Reaction Wrench from Wall: ["<<reactionWrench.toString()<<"]"<<std::endl;
    EXPECT_DOUBLE_EQ(reactionWrench[4], Kw(4,4)*M_PI_2/2.0);

    TEE = TEE.eye();
    TEE_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(TEE, TEE_KDL);
    TEE_KDL.M.DoRotZ(M_PI_2/4.0);
    cartesian_utils::fromKDLFrameToYARPMatrix(TEE_KDL, TEE);

    reactionWrench = computeWallForce(TW, TEE, Kw);

    std::cout<<"Reaction Wrench from Wall: ["<<reactionWrench.toString()<<"]"<<std::endl;
    EXPECT_DOUBLE_EQ(reactionWrench[5], -Kw(5,5)*M_PI_2/4.0);
}

TEST_F(testInteractionTask, testInteractionTask_wrench)
{
    ofstream file;
    file.open("TestInteraction_testInteractionTask_wrench_error_wrench.m");


    std::string urdf_file = std::string(OPENSOT_TESTS_ROBOTS_DIR) + "bigman/bigman.urdf";
    std::string srdf_file = std::string(OPENSOT_TESTS_ROBOTS_DIR) + "bigman/bigman.srdf";

    iDynUtils _robot("bigman", urdf_file, srdf_file);

    yarp::sig::Vector q = getGoodInitialPosition(_robot);
    _robot.updateiDyn3Model(q, true);

    std::string distal_link = "l_arm_ft";
    std::string ft_sensor_link = "l_arm_ft";
    std::string base_link = "torso";

    _robot.switchAnchor(base_link);

    // We consider the robot in contact with the wall, with 0 contact force:
    yarp::sig::Matrix base_link_T_Wall = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(base_link),
                                                             _robot.iDyn3_model.getLinkIndex(distal_link));

    std::cout<<"Position of the wall in base_link:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(base_link_T_Wall);
    std::cout<<std::endl;

    // The robot has a certain Compliance:

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) = 1E-2;
        C(i+3, i+3) = 1E-2;}

    // We compute the applied contact force as:
    yarp::sig::Vector base_link_WallWrench = -1.0*computeWallForce(base_link_T_Wall, base_link_T_Wall, yarp::math::pinv(C));
    // We have to report the wrench in sensor frame
    KDL::Wrench base_link_WallWrench_KDL; cartesian_utils::fromYarpVectortoKDLWrench(base_link_WallWrench, base_link_WallWrench_KDL);
    yarp::sig::Matrix base_link_T_sensor = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(base_link),
                                                             _robot.iDyn3_model.getLinkIndex(ft_sensor_link));
    KDL::Frame base_link_T_sensor_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(base_link_T_sensor, base_link_T_sensor_KDL);
    KDL::Wrench sensor_WallWrench_KDL = base_link_T_sensor_KDL.Inverse()*base_link_WallWrench_KDL;
    yarp::sig::Vector sensor_WallWrench; cartesian_utils::fromKDLWrenchtoYarpVector(sensor_WallWrench_KDL, sensor_WallWrench);

    // And we set as a feedback
    for(unsigned int i = 0; i < 4; ++i)
        _robot.iDyn3_model.setSensorMeasurement(i, sensor_WallWrench);

    ASSERT_EQ(yarp::math::norm(sensor_WallWrench),0.0);

    Interaction::Ptr interactionTask(new Interaction("interaction::l_wrist", q, _robot,
                                                     distal_link,
                                                     base_link,
                                                     ft_sensor_link));

    interactionTask->setCompliance(C);

    EXPECT_TRUE(C == interactionTask->getCompliance());

    Postural::Ptr posturalTask(new Postural(q));

    int T = 1000;
    VelocityLimits::Ptr joint_velocity_limits(new VelocityLimits(0.1, (double)(1.0/T), q.size()));

    JointLimits::Ptr joint_limits(new JointLimits(q,
                                                _robot.iDyn3_model.getJointBoundMax(),
                                                _robot.iDyn3_model.getJointBoundMin()));

    //Create the SoT
    std::vector< OpenSoT::Task<Matrix, Vector>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(interactionTask);
    stack_of_tasks.push_back(posturalTask);

    std::list< OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);


    std::cout<<distal_link<<" actual position in "<<base_link<<" @t0: "<<std::endl;
    cartesian_utils::printHomogeneousTransform(interactionTask->getActualPose());
    std::cout <<"Applied wrench@t0: ["<<interactionTask->getActualWrench().toString()<<"]"<<std::endl;
    ASSERT_EQ(yarp::math::norm(interactionTask->getActualWrench()),0.0);

    yarp::sig::Vector base_WallWrenchDesired = interactionTask->getActualWrench();
    base_WallWrenchDesired[0] = 10.0;
    base_WallWrenchDesired[1] = 5.0;
    base_WallWrenchDesired[2] = -1.0;
    base_WallWrenchDesired[3] = 1.5;
    base_WallWrenchDesired[4] = -1.1;
    base_WallWrenchDesired[5] = 1.2;
    interactionTask->setReferenceWrench(base_WallWrenchDesired);

    yarp::sig::Vector dq(q.size(), 0.0);
    unsigned int max_iterations = 10000;
    file<<"error_wrench = ["<<std::endl;
    for(unsigned int i = 0; i < max_iterations; ++i)
    {
        _robot.updateiDyn3Model(q, true);

        yarp::sig::Matrix base_link_T_EE = interactionTask->getActualPose();
        std::cout<<"\n\n---@t"<<double(i)/T<<"------------\n";
        std::cout<<distal_link<<" actual position in "<<base_link<<std::endl;
        cartesian_utils::printHomogeneousTransform(base_link_T_EE);
        base_link_WallWrench = -1.0*computeWallForce(base_link_T_Wall, base_link_T_EE, pinv(C));
        std::cout<<"Measured Applied Wall Wrench in "<<base_link<<"frame: ["<<base_link_WallWrench.toString()<<"]"<<std::endl;

        // We have to report the wrench in sensor frame
        KDL::Wrench base_link_WallWrench_KDL; cartesian_utils::fromYarpVectortoKDLWrench(base_link_WallWrench, base_link_WallWrench_KDL);
        yarp::sig::Matrix base_link_T_sensor = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(base_link),
                                                                 _robot.iDyn3_model.getLinkIndex(ft_sensor_link));
        KDL::Frame base_link_T_sensor_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(base_link_T_sensor, base_link_T_sensor_KDL);
        KDL::Wrench sensor_WallWrench_KDL = base_link_T_sensor_KDL.Inverse()*base_link_WallWrench_KDL;
        yarp::sig::Vector sensor_WallWrench; cartesian_utils::fromKDLWrenchtoYarpVector(sensor_WallWrench_KDL, sensor_WallWrench);
        std::cout<<"Measured Applied Wall Wrench in sensor frame: ["<<sensor_WallWrench.toString()<<"]"<<std::endl;

         for(unsigned int i = 0; i < _robot.iDyn3_model.getNrOfFTSensors(); ++i)
             _robot.iDyn3_model.setSensorMeasurement(i, sensor_WallWrench);



         interactionTask->update(q);
         joint_constraints->update(q);

         std::cout <<"Applied wrench in base_link: ["<<interactionTask->getActualWrench().toString()<<"]"<<std::endl;
         std::cout<<"Force error: ["<<interactionTask->forceError.toString()<<"]"<<std::endl;
         std::cout<<"Torque error: ["<<interactionTask->torqueError.toString()<<"]"<<std::endl;

         file<<interactionTask->forceError.toString()<<" "<<interactionTask->torqueError.toString()<<std::endl;

         sot.solve(dq);
         q += dq;
    }
    file<<"]"<<std::endl;
    file.close();

    EXPECT_NEAR(interactionTask->getActualWrench()[0], base_WallWrenchDesired[0], 1E-1);
    EXPECT_NEAR(interactionTask->getActualWrench()[1], base_WallWrenchDesired[1], 1E-1);
    EXPECT_NEAR(interactionTask->getActualWrench()[2], base_WallWrenchDesired[2], 1E-1);
    EXPECT_NEAR(interactionTask->getActualWrench()[3], base_WallWrenchDesired[3], 2E-1);
    EXPECT_NEAR(interactionTask->getActualWrench()[4], base_WallWrenchDesired[4], 2E-1);
    EXPECT_NEAR(interactionTask->getActualWrench()[5], base_WallWrenchDesired[5], 2E-1);

}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
