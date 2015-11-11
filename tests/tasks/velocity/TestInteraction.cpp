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
#include <idynutils/RobotUtils.h>

using namespace yarp::math;
using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

#define VISUALIZE_SIMULATION true

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

yarp::sig::Vector getGoodInitialPosition2(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 0.0 * M_PI/180.0;
    arm[1] = 0.0 * M_PI/180.0;
    arm[3] = -90.0 * M_PI/180.0;
    arm[6] = 1.3;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    arm[6] = -arm[6];
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

using namespace OpenSoT;

TEST_F(testInteractionTask, testInteractionTask_wrenchSimulation) {
//    // Start YARP Server
//    tests_utils::startYarpServer();
//    // Load a world
//    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman_brick_wall.world";
//    if(VISUALIZE_SIMULATION)
//        tests_utils::startGazebo(world_path);
//    else
//        tests_utils::startGZServer(world_path);
    sleep(4);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition2(coman_robot.idynutils);

    //Homing
    coman_robot.setPositionMode();
    double speed = 0.8;
    yarp::sig::Vector legs_speed(6,speed);
    legs_speed[3] = 2.0*legs_speed[3];
    coman_robot.left_leg.setReferenceSpeeds(legs_speed);
    coman_robot.right_leg.setReferenceSpeeds(legs_speed);
    coman_robot.left_arm.setReferenceSpeed(speed);
    coman_robot.right_arm.setReferenceSpeed(speed);
    coman_robot.torso.setReferenceSpeed(speed);
    coman_robot.move(q);

    //Set Up SoT
    sleep(10);
    coman_robot.setPositionDirectMode();
    sleep(2);

    std::vector<iDynUtils::ft_measure> _ft_measurements;
    RobotUtils::ftPtrMap ft_sensors = coman_robot.getftSensors();
    for(RobotUtils::ftPtrMap::iterator it = ft_sensors.begin();
        it != ft_sensors.end(); it++)
    {
        iDynUtils::ft_measure ft_measurement;
        ft_measurement.first = it->second->getReferenceFrame();
        yarp::sig::Vector dummy_measure(6 ,0.0);
        ft_measurement.second = dummy_measure;

        _ft_measurements.push_back(ft_measurement);
    }
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);
    coman_robot.idynutils.setFloatingBaseLink(coman_robot.idynutils.left_leg.end_effector_name);



//    yarp::sig::Vector wrench_d_l_wrist(6,0.0);
//    wrench_d_l_wrist[0] = 20.0; wrench_d_l_wrist[1] = -2.0; wrench_d_l_wrist[2] = 10.0;
//    wrench_d_l_wrist[3] = 2.0; wrench_d_l_wrist[4] = -2.0; wrench_d_l_wrist[5] = -2.0;

    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

    double dT = 0.005;
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(0.3, dT,q.size()));

    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));

    RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
    std::vector<yarp::sig::Vector> filter_ft;
    for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
        _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];
        filter_ft.push_back(-1.0*ft_readings[_ft_measurements[i].first]);
    }
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) = 1E-5;
        C(i+3,i+3) = 1E-4;
    }


    tasks::velocity::Interaction::Ptr interaction_rwrist_task(
                new tasks::velocity::Interaction("interaction::r_wrist",
                                q, coman_robot.idynutils, "r_wrist", "world", "r_wrist"));
    std::vector<bool> active_joint_mask = interaction_rwrist_task->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[coman_robot.idynutils.left_leg.joint_numbers[i]] = false;
    interaction_rwrist_task->setActiveJointsMask(active_joint_mask);
    yarp::sig::Matrix W = interaction_rwrist_task->getWeight();
    W(3,3) = 0.1; W(4,4) = 0.1; W(5,5) = 0.1;
    interaction_rwrist_task->setWeight(W);
    interaction_rwrist_task->setCompliance(C);
    yarp::sig::Vector wrench_d_r_wrist(6,0.0);
    wrench_d_r_wrist[0] = 25.0; wrench_d_r_wrist[1] = 5.0; wrench_d_r_wrist[2] = 5.0;
    wrench_d_r_wrist[3] = 0.0; wrench_d_r_wrist[4] = 5.0; wrench_d_r_wrist[5] = 5.0;
    interaction_rwrist_task->setReferenceWrench(wrench_d_r_wrist);
    interaction_rwrist_task->update(q);

//    tasks::velocity::Interaction::Ptr interaction_lwrist_task(
//                new tasks::velocity::Interaction("interaction::l_wrist",
//                                q, coman_robot.idynutils, "l_wrist", "Waist", "l_wrist"));
//    interaction_lwrist_task->setCompliance(C);
//    interaction_lwrist_task->setReferenceWrench(wrench_d_l_wrist);
//    interaction_lwrist_task->update(q);


    std::list<tasks::velocity::Cartesian::TaskPtr> aggregated_list;
    aggregated_list.push_back(interaction_rwrist_task);
    //aggregated_list.push_back(interaction_lwrist_task);
    Task<Matrix, Vector>::TaskPtr taskAggregatedHighest =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(aggregated_list,q.size()));

    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskAggregatedHighest);
    stack_of_tasks.push_back(postural_task);

    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
    sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, 1E10));

    yarp::sig::Vector dq(q.size(), 0.0);
    int steps = 3000;
    std::vector<yarp::sig::Vector> wrench_measured;
    std::vector<yarp::sig::Vector> wrench_desired;
    wrench_measured.reserve(steps);
    wrench_desired.reserve(steps);

    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();



        ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
            //filter_ft[i] = -1.0*ft_readings[_ft_measurements[i].first];
            filter_ft[i] += (-1.0*ft_readings[_ft_measurements[i].first]-filter_ft[i])*0.25;
            _ft_measurements[i].second = filter_ft[i];}
        coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);

        bounds->update(q);
        taskAggregatedHighest->update(q);
        postural_task->update(q);


//        wrench_desired.push_back(yarp::math::cat(interaction_lwrist_task->getReferenceWrench(),
//                                                 interaction_rwrist_task->getReferenceWrench()));
//        wrench_measured.push_back(yarp::math::cat(
//            interaction_lwrist_task->getActualWrench(), interaction_rwrist_task->getActualWrench()));
        wrench_desired.push_back(interaction_rwrist_task->getReferenceWrench());
        wrench_measured.push_back(interaction_rwrist_task->getActualWrench());


        if(sot->solve(dq)){
            q += dq;}
        coman_robot.move(q);



        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
        else
            std::cout<<"we are too slow!"<<std::endl;
    }
    std::ofstream file2;
    std::string file_name2 = "testCoMForce_wrenchMeasured4.m";
    file2.open(file_name2);
    file2<<"wrench_measured_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_measured.size(); ++i)
        file2<<wrench_measured[i].toString()<<std::endl;
    file2<<"];"<<std::endl;
    file2.close();

    std::ofstream file5;
    std::string file_name5 = "testCoMForce_wrenchDesired4.m";
    file5.open(file_name5);
    file5<<"wrench_desired_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_desired.size(); ++i)
        file5<<wrench_desired[i].toString()<<std::endl;
    file5<<"];"<<std::endl;
    file5.close();

    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
