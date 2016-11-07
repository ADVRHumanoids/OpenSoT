#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <ros/ros.h>


using namespace yarp::math;

namespace{

class testForceCoM : public ::testing::Test {
 protected:

  testForceCoM()
  {

  }

  virtual ~testForceCoM() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


};

yarp::sig::Vector getGoodInitialPosition1(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = -90.0 * M_PI/180.0;
    arm[1] = 0.0 * M_PI/180.0;
    arm[3] = 0.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 0.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = 0.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

TEST_F(testForceCoM, testForceCoM_StaticCase) {
    iDynUtils coman("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    yarp::sig::Vector q = getGoodInitialPosition1(coman);

    coman.updateiDyn3Model(q, true);

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("r_sole");
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("LSoftHand");

    std::vector<std::string>::iterator it;
    for(it = links_in_contact.begin();
        it != links_in_contact.end(); it++)
        std::cout<<"link in contact "<<":"<<*it<<std::endl;


    Eigen::VectorXd contact_wrenches_d(6*links_in_contact.size());
    contact_wrenches_d.setZero(contact_wrenches_d.rows());
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
        new OpenSoT::tasks::force::CoM(contact_wrenches_d, links_in_contact, coman));
    force_com_task->update(contact_wrenches_d);

    Eigen::MatrixXd A = force_com_task->getA();
    EXPECT_EQ(A.rows(), 6);
    EXPECT_EQ(A.cols(), 6*links_in_contact.size());
    std::cout<<"A = [ "<<A<<" ]"<<std::endl;

    Eigen::VectorXd b = force_com_task->getb();
    EXPECT_DOUBLE_EQ(b.rows(), 6);
    std::cout<<"b = [ "<<b<<" ]"<<std::endl;

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(force_com_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E3));
    std::cout<<"Solver started"<<std::endl;
    sot->solve(contact_wrenches_d);
    std::cout<<"contact_wrenches_d = [ "<<contact_wrenches_d<<" ]"<<std::endl;

    yarp::sig::Matrix M(6+coman.iDyn3_model.getNrOfDOFs(),
                        6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M);
    double m = M(0,0);

    EXPECT_NEAR(contact_wrenches_d[2] + contact_wrenches_d[8]
            + contact_wrenches_d[14], m*9.81, 1E-6);

    Eigen::VectorXd Ax = A*contact_wrenches_d;
    std::cout<<"A*x = \n"<<Ax<<std::endl;
    std::cout<<"b = \n"<<b<<std::endl;

    for(unsigned int i = 0; i < b.rows(); ++i)
        EXPECT_NEAR(Ax[i],b[i], 1E-6);

}

#if OPENSOT_COMPILE_SIMULATION_TESTS

using namespace yarp::sig;
using namespace OpenSoT;

TEST_F(testForceCoM, testForceCoM2) {
//    int argc = 0;
//    char *argv[] = {""};
//    ros::init(argc, argv, "testForceCoM2");
//    //ROS Related stuffs
//    ros::NodeHandle n;
//    ros::Publisher new_world_pub;
//    new_world_pub = n.advertise<geometry_msgs::TransformStamped>("/anchor_to_world_pose", 1000);


    // Start YARP Server
    tests_utils::startYarpServer();
    // Load a world
    //std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/block.world";
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);
    sleep(4);

//    //WAIT
//    cin.get();

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);
    q[coman_robot.idynutils.left_leg.joint_numbers[5]] += -20.0*M_PI/180.0;
    q[coman_robot.idynutils.right_leg.joint_numbers[5]] += -20.0*M_PI/180.0;

    //Homing
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

    sleep(5);
    legs_speed = legs_speed/5.0;
    coman_robot.left_leg.setReferenceSpeeds(legs_speed);
    coman_robot.right_leg.setReferenceSpeeds(legs_speed);

    q[coman_robot.idynutils.left_leg.joint_numbers[5]] += -10.0*M_PI/180.0;
    q[coman_robot.idynutils.right_leg.joint_numbers[5]] += -10.0*M_PI/180.0;
    coman_robot.move(q);
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);

    //Set Up SoT
    sleep(10);
    coman_robot.setPositionDirectMode();
    sleep(2);

    std::string anchor; KDL::Frame anchor_T_World;
    coman_robot.idynutils.getWorldPose(anchor_T_World, anchor);
    anchor_T_World.M.DoRotY(20.0*M_PI/180.0);
    coman_robot.idynutils.setAnchor_T_World(anchor_T_World);
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);

//    //Publish world in ros
//    geometry_msgs::TransformStamped T;
//    T.header.frame_id = "l_sole";
//    T.child_frame_id = "world";
//    KDL::Frame anchorKDL = coman_robot.idynutils.getAnchor_T_World();
//    double qx,qy,qz,qw;
//    anchorKDL.M.GetQuaternion(qx,qy,qz,qw);
//    T.transform.rotation.x = qx;
//    T.transform.rotation.y = qy;
//    T.transform.rotation.z = qz;
//    T.transform.rotation.w = qw;
//    T.transform.translation.x = anchorKDL.p[0];
//    T.transform.translation.y = anchorKDL.p[1];
//    T.transform.translation.z = anchorKDL.p[2];
//    new_world_pub.publish(T);
//    cin.get();
//    ///////////////////////////////////////

    std::vector<std::string> ft_in_contact;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(coman_robot.idynutils.getForceTorqueFrameNames(),
               std::list<std::string>{std::begin(coman_robot.idynutils.getLinksInContact()),
                                        std::end(coman_robot.idynutils.getLinksInContact())},
                                        coman_robot.idynutils,
                                        ft_in_contact);


    //SET UP FORCE OPTIMIZATION
    yarp::sig::Vector wrench_d(12,0.0);
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
                new OpenSoT::tasks::force::CoM(wrench_d, coman_robot.idynutils));
    force_com_task->setLambda(400.0,40.0);

    std::cout<<"INITIAL REF = ["<<force_com_task->getReference().toString()<<std::endl;
    std::cout<<"INITIAL COM POSE = ["<<coman_robot.idynutils.iDyn3_model.getCOM().toString()<<std::endl;


    std::map<std::string, double> mu;
    KDL::Rotation R_KDL; R_KDL.DoRotY(-20.0*M_PI/180.0);
    yarp::sig::Matrix R(3,3); R.eye();
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            R(i,j) = R_KDL(i,j);


    std::map<std::string, yarp::sig::Matrix> world_R_surfaces;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
    {
        mu[ft_in_contact[i]] = 0.5;
        world_R_surfaces[ft_in_contact[i]] = R;
    }
    OpenSoT::constraints::force::FrictionCone::Ptr friction_cones_constraint(
                new OpenSoT::constraints::force::FrictionCone(wrench_d, coman_robot.idynutils,
                                                              mu,world_R_surfaces));

    force_com_task->getConstraints().push_back(friction_cones_constraint);

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks_force;
    stack_of_tasks_force.push_back(force_com_task);
    OpenSoT::solvers::QPOases_sot::Ptr sot_force(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks_force,2E5));
    sot_force->solve(wrench_d);

    //This is the wrench in sensor frame expressed in world, we have to tranform it!
    yarp::sig::Matrix world_T_ft_l = coman_robot.idynutils.iDyn3_model.getPosition(
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
    yarp::sig::Matrix world_T_ft_r = coman_robot.idynutils.iDyn3_model.getPosition(
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
    KDL::Frame world_T_ft_lKDL, world_T_ft_rKDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_l, world_T_ft_lKDL);
    cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_r, world_T_ft_rKDL);

    yarp::sig::Vector wrench_d_lankle = -1.0*yarp::math::cat(
                wrench_d.subVector(0,2),wrench_d.subVector(6,8));
    yarp::sig::Vector wrench_d_rankle = -1.0*yarp::math::cat(
                wrench_d.subVector(3,5),wrench_d.subVector(9,11));
    std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in world"<<std::endl;
    std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in world"<<std::endl;

    KDL::Wrench wrench_d_lankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
    KDL::Wrench wrench_d_rankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);
    KDL::Frame base_link_T_ft_l = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
    KDL::Frame base_link_T_ft_r = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
    wrench_d_lankle_KDL = base_link_T_ft_l*world_T_ft_lKDL.Inverse()*wrench_d_lankle_KDL;
    wrench_d_rankle_KDL = base_link_T_ft_r*world_T_ft_rKDL.Inverse()*wrench_d_rankle_KDL;
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);

    std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in Waist"<<std::endl;
    std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in Waist"<<std::endl;

    //cin.get();

    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

    double dT = 0.001;
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(M_PI_2, dT,q.size()));

    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));

//    std::cout<<"INITIAL REF = ["<<force_com_task->getReference().toString()<<std::endl;
//    std::cout<<"INITIAL COM POSE = ["<<coman_robot.idynutils.iDyn3_model.getCOM().toString()<<std::endl;


    RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
    std::vector<yarp::sig::Vector> filter_ft;
    for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
        _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];
        filter_ft.push_back(-1.0*ft_readings[_ft_measurements[i].first]);}
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, false);

//    std::cout<<"INITIAL REF = ["<<force_com_task->getReference().toString()<<std::endl;
//    std::cout<<"INITIAL COM POSE = ["<<coman_robot.idynutils.iDyn3_model.getCOM().toString()<<std::endl;

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i)
    {
        C(i,i) *= 1E-6;
        C(i+3,i+3) *= 1E-7;
    }

    tasks::velocity::Interaction::Ptr interaction_lankle_task(
                new tasks::velocity::Interaction("interaction::l_ankle",
                                q, coman_robot.idynutils, "l_ankle", "Waist", "l_ankle"));
    interaction_lankle_task->setCompliance(C);
    interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
    interaction_lankle_task->update(q);
//    std::cout<<"actual_wrench_lankle = ["<<interaction_lankle_task->getActualWrench().toString()<<"] expressed in Waist"<<std::endl;
//    std::cout<<"error_wrench_lankle = ["<<interaction_lankle_task->getWrenchError().toString()<<"]"<<std::endl;

    tasks::velocity::Interaction::Ptr interaction_rankle_task(
                new tasks::velocity::Interaction("interaction::r_ankle",
                                q, coman_robot.idynutils, "r_ankle", "Waist", "r_ankle"));
    interaction_rankle_task->setCompliance(C);
    interaction_rankle_task->setReferenceWrench(wrench_d_rankle);
    interaction_rankle_task->update(q);
//    std::cout<<"actual_wrench_lankle = ["<<interaction_rankle_task->getActualWrench().toString()<<"] expressed in Waist"<<std::endl;
//    std::cout<<"error_wrench_rankle = ["<<interaction_rankle_task->getWrenchError().toString()<<"]"<<std::endl;



    std::list<tasks::velocity::Cartesian::TaskPtr> aggregated_list;
    aggregated_list.push_back(interaction_lankle_task);
    aggregated_list.push_back(interaction_rankle_task);
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
    yarp::sig::Vector wrench(wrench_d.size(), 0.0);
    int steps = 3*int(1.5*M_PI*1000);
    std::vector<yarp::sig::Vector> wrench_error;
    std::vector<yarp::sig::Vector> wrench_measured;
    std::vector<yarp::sig::Vector> com;
    wrench_error.reserve(steps);
    wrench_measured.reserve(steps);
    com.reserve(steps);
    yarp::sig::Vector ref = force_com_task->getReference();
    std::cout<<"INITIAL REF = ["<<ref.toString()<<std::endl;
    std::cout<<"INITIAL COM POSE = ["<<coman_robot.idynutils.iDyn3_model.getCOM().toString()<<std::endl;


    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();

//        //Publish world in ros
//        anchorKDL = coman_robot.idynutils.getAnchor_T_World();
//        anchorKDL.M.GetQuaternion(qx,qy,qz,qw);
//        T.transform.rotation.x = qx;
//        T.transform.rotation.y = qy;
//        T.transform.rotation.z = qz;
//        T.transform.rotation.w = qw;
//        T.transform.translation.x = anchorKDL.p[0];
//        T.transform.translation.y = anchorKDL.p[1];
//        T.transform.translation.z = anchorKDL.p[2];
//        new_world_pub.publish(T);
//        ///////////////////////////////////////





        if(i <= M_PI*1000){
            yarp::sig::Vector goal = ref;
            goal(2) += (-0.15)*std::sin((i+M_PI)/1000.);
            yarp::sig::Vector goal_before = ref;
            if(i == 0)
                goal_before(2) = 0.0;
            else
                goal_before(2) += (-0.15)*std::sin((i-1+M_PI)/1000.);
            yarp::sig::Vector v(3,0.0);
            v(2)= (goal(2)-goal_before(2))/dT;
            force_com_task->setReference(goal,v);
            }


        ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
            filter_ft[i] += (-1.0*ft_readings[_ft_measurements[i].first]-filter_ft[i])*0.1;
            _ft_measurements[i].second = filter_ft[i];}
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);

        com.push_back(coman_robot.idynutils.iDyn3_model.getCOM());

        force_com_task->update(wrench_d);
        friction_cones_constraint->update(wrench_d);

//        std::cout<<std::endl;
//        std::cout<<"force_com_task actual position"<<force_com_task->getActualPosition().toString()<<std::endl;
//        std::cout<<"force_com_task reference"<<force_com_task->getReference().toString()<<std::endl;
//        std::cout<<"force_com_task position error: ["<<force_com_task->getError().toString()<<"]"<<std::endl;
//        std::cout<<"force_com_task b: ["<<force_com_task->getb().toString()<<"]"<<std::endl;
//        std::cout<<"force_com_task A: ["<<force_com_task->getA().toString()<<"]"<<std::endl;



        if(sot_force->solve(wrench))
            wrench_d = wrench;
        else
            std::cout<<"ERROR FORCE Optimization"<<std::endl;

        world_T_ft_l = coman_robot.idynutils.iDyn3_model.getPosition(
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
        world_T_ft_r = coman_robot.idynutils.iDyn3_model.getPosition(
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
        cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_l, world_T_ft_lKDL);
        cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_r, world_T_ft_rKDL);

        wrench_d_lankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(0,2),wrench_d.subVector(6,8));
        wrench_d_rankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(3,5),wrench_d.subVector(9,11));
//        std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in world"<<std::endl;
//        std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in world"<<std::endl;

        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);
        base_link_T_ft_l = coman_robot.idynutils.iDyn3_model.getPositionKDL(
            coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
        base_link_T_ft_r = coman_robot.idynutils.iDyn3_model.getPositionKDL(
            coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
        wrench_d_lankle_KDL = base_link_T_ft_l*world_T_ft_lKDL.Inverse()*wrench_d_lankle_KDL;
        wrench_d_rankle_KDL = base_link_T_ft_r*world_T_ft_rKDL.Inverse()*wrench_d_rankle_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);

//        std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in Waist"<<std::endl;
//        std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in Waist"<<std::endl;


        //cin.get();
        interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
        interaction_rankle_task->setReferenceWrench(wrench_d_rankle);

        bounds->update(q);
        taskAggregatedHighest->update(q);
        postural_task->update(q);



        wrench_error.push_back(yarp::math::cat(
            interaction_lankle_task->getWrenchError(), interaction_rankle_task->getWrenchError()));
        wrench_measured.push_back(yarp::math::cat(
            interaction_lankle_task->getActualWrench(), interaction_rankle_task->getActualWrench()));


        if(sot->solve(dq)){
            q += dq;}
        coman_robot.move(q);



        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
    }
    std::ofstream file1;
    std::string file_name = "testCoMForce_wrenchError.m";
    file1.open(file_name);
    file1<<"wrench_error_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_error.size(); ++i)
        file1<<wrench_error[i].toString()<<std::endl;
    file1<<"];"<<std::endl;
    file1.close();

    std::ofstream file2;
    std::string file_name2 = "testCoMForce_wrenchMeasured.m";
    file2.open(file_name2);
    file2<<"wrench_measured_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_measured.size(); ++i)
        file2<<wrench_measured[i].toString()<<std::endl;
    file2<<"];"<<std::endl;
    file2.close();

    std::ofstream file3;
    std::string file_name3 = "testCoMForce_com.m";
    file3.open(file_name3);
    file3<<"com = ["<<std::endl;
    for(unsigned int i = 0; i < com.size(); ++i)
        file3<<com[i].toString()<<std::endl;
    file3<<"];"<<std::endl;
    file3.close();

    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

TEST_F(testForceCoM, testForceCoM3) {
    // Start YARP Server
    tests_utils::startYarpServer();
    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);
    sleep(4);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

    //Homing
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


    std::vector<std::string> ft_in_contact;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(coman_robot.idynutils.getForceTorqueFrameNames(),
               std::list<std::string>{std::begin(coman_robot.idynutils.getLinksInContact()),
                                        std::end(coman_robot.idynutils.getLinksInContact())},
                                        coman_robot.idynutils,
                                        ft_in_contact);


    //SET UP FORCE OPTIMIZATION
    yarp::sig::Vector wrench_d(12,0.0);
    OpenSoT::tasks::force::CoM::Ptr force_com_task(
                new OpenSoT::tasks::force::CoM(wrench_d, coman_robot.idynutils));
    force_com_task->setLambda(400.0,40.0);


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks_force;
    stack_of_tasks_force.push_back(force_com_task);
    OpenSoT::solvers::QPOases_sot::Ptr sot_force(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks_force,2E10));
    sot_force->solve(wrench_d);

    //This is the wrench in sensor frame expressed in world, we have to tranform it!
    yarp::sig::Matrix world_T_ft_l = coman_robot.idynutils.iDyn3_model.getPosition(
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
    yarp::sig::Matrix world_T_ft_r = coman_robot.idynutils.iDyn3_model.getPosition(
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
    KDL::Frame world_T_ft_lKDL, world_T_ft_rKDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_l, world_T_ft_lKDL);
    cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_r, world_T_ft_rKDL);

    yarp::sig::Vector wrench_d_lankle = -1.0*yarp::math::cat(
                wrench_d.subVector(0,2),wrench_d.subVector(6,8));
    yarp::sig::Vector wrench_d_rankle = -1.0*yarp::math::cat(
                wrench_d.subVector(3,5),wrench_d.subVector(9,11));
    std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in world"<<std::endl;
    std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in world"<<std::endl;

    KDL::Wrench wrench_d_lankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
    KDL::Wrench wrench_d_rankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);
    KDL::Frame base_link_T_ft_l = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
    KDL::Frame base_link_T_ft_r = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
    wrench_d_lankle_KDL = base_link_T_ft_l*world_T_ft_lKDL.Inverse()*wrench_d_lankle_KDL;
    wrench_d_rankle_KDL = base_link_T_ft_r*world_T_ft_rKDL.Inverse()*wrench_d_rankle_KDL;
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);

    std::cout<<"wrench_d_lankle = ["<<wrench_d_lankle.toString()<<"] expressed in Waist"<<std::endl;
    std::cout<<"wrench_d_rankle = ["<<wrench_d_rankle.toString()<<"] expressed in Waist"<<std::endl;

    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

    double dT = 0.01;
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(M_PI_2, dT,q.size()));

    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));

    RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
    std::vector<yarp::sig::Vector> filter_ft;
    for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
        _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];
        filter_ft.push_back(-1.0*ft_readings[_ft_measurements[i].first]);}
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i)
    {
        C(i,i) *= 1.2E-6;
        C(i+3,i+3) *= 1.2E-7;
    }

    tasks::velocity::Interaction::Ptr interaction_lankle_task(
                new tasks::velocity::Interaction("interaction::l_ankle",
                                q, coman_robot.idynutils, "l_ankle", "Waist", "l_ankle"));
    interaction_lankle_task->setCompliance(C);
    interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
    interaction_lankle_task->update(q);
    std::cout<<"actual_wrench_lankle = ["<<interaction_lankle_task->getActualWrench().toString()<<"] expressed in Waist"<<std::endl;
    std::cout<<"error_wrench_lankle = ["<<interaction_lankle_task->getWrenchError().toString()<<"]"<<std::endl;

    tasks::velocity::Interaction::Ptr interaction_rankle_task(
                new tasks::velocity::Interaction("interaction::r_ankle",
                                q, coman_robot.idynutils, "r_ankle", "Waist", "r_ankle"));
    interaction_rankle_task->setCompliance(C);
    interaction_rankle_task->setReferenceWrench(wrench_d_rankle);
    interaction_rankle_task->update(q);
    std::cout<<"actual_wrench_lankle = ["<<interaction_rankle_task->getActualWrench().toString()<<"] expressed in Waist"<<std::endl;
    std::cout<<"error_wrench_rankle = ["<<interaction_rankle_task->getWrenchError().toString()<<"]"<<std::endl;



    std::list<tasks::velocity::Cartesian::TaskPtr> aggregated_list;
    aggregated_list.push_back(interaction_lankle_task);
    aggregated_list.push_back(interaction_rankle_task);
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
    yarp::sig::Vector wrench(wrench_d.size(), 0.0);
    int steps = 3*int(1.5*M_PI*200);
    std::vector<yarp::sig::Vector> wrench_measured;
    std::vector<yarp::sig::Vector> wrench_desired;
    std::vector<yarp::sig::Vector> com;
    std::vector<yarp::sig::Vector> com_d;
    wrench_measured.reserve(steps);
    wrench_desired.reserve(steps);
    com.reserve(steps);
    yarp::sig::Vector ref = force_com_task->getReference();
    std::cout<<"INITIAL REF = ["<<ref.toString()<<std::endl;
    std::cout<<"INITIAL COM POSE = ["<<coman_robot.idynutils.iDyn3_model.getCOM().toString()<<std::endl;

    yarp::sig::Vector vel_before;
    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();

        if(i <= M_PI*400){
            yarp::sig::Vector goal = ref;
            goal(1) += (-0.05)*std::sin((i+M_PI)/400.);
            goal(2) += (-0.1)*std::sin((i+M_PI)/400.);

            yarp::sig::Vector goal_before = ref;
            if(i == 0){
                goal_before(1) = 0.0;
                goal_before(2) = 0.0;}
            else{
                goal_before(1) += (-0.05)*std::sin((i-1+M_PI)/400.);
                goal_before(2) += (-0.1)*std::sin((i-1+M_PI)/400.);}

            yarp::sig::Vector v(3,0.0);
            v(1)= (goal(1)-goal_before(1))/dT;
            v(2)= (goal(2)-goal_before(2))/dT;

            yarp::sig::Vector a(3,0.0);
            if(i == 0)
                a = v/dT;
            else
                a = (v-vel_before)/dT;
            vel_before = v;

            force_com_task->setReference(goal,v,a);
            com_d.push_back(goal);
            }


        ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
            filter_ft[i] += (-1.0*ft_readings[_ft_measurements[i].first]-filter_ft[i])*0.25;
            _ft_measurements[i].second = filter_ft[i];}
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);

//        if(i == int((M_PI*1000)/2)){
//            yarp::sig::Vector com = coman_robot.idynutils.iDyn3_model.getCOM();
//            EXPECT_NEAR
//        }

        com.push_back(coman_robot.idynutils.iDyn3_model.getCOM());

        force_com_task->update(wrench_d);



        if(sot_force->solve(wrench))
           wrench_d = wrench;

        world_T_ft_l = coman_robot.idynutils.iDyn3_model.getPosition(
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
        world_T_ft_r = coman_robot.idynutils.iDyn3_model.getPosition(
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
        cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_l, world_T_ft_lKDL);
        cartesian_utils::fromYARPMatrixtoKDLFrame(world_T_ft_r, world_T_ft_rKDL);

        wrench_d_lankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(0,2),wrench_d.subVector(6,8));
        wrench_d_rankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(3,5),wrench_d.subVector(9,11));

        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);
        base_link_T_ft_l = coman_robot.idynutils.iDyn3_model.getPositionKDL(
            coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[0]));
        base_link_T_ft_r = coman_robot.idynutils.iDyn3_model.getPositionKDL(
            coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
            coman_robot.idynutils.iDyn3_model.getLinkIndex(ft_in_contact[1]));
        wrench_d_lankle_KDL = base_link_T_ft_l.M*world_T_ft_lKDL.Inverse().M*wrench_d_lankle_KDL;
        wrench_d_rankle_KDL = base_link_T_ft_r.M*world_T_ft_rKDL.Inverse().M*wrench_d_rankle_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);

        interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
        interaction_rankle_task->setReferenceWrench(wrench_d_rankle);

        bounds->update(q);
        taskAggregatedHighest->update(q);
        postural_task->update(q);


        wrench_desired.push_back(yarp::math::cat(interaction_lankle_task->getReferenceWrench(),
                                                 interaction_rankle_task->getReferenceWrench()));
        wrench_measured.push_back(yarp::math::cat(
            interaction_lankle_task->getActualWrench(), interaction_rankle_task->getActualWrench()));


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
    std::string file_name2 = "testCoMForce_wrenchMeasured3.m";
    file2.open(file_name2);
    file2<<"wrench_measured_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_measured.size(); ++i)
        file2<<wrench_measured[i].toString()<<std::endl;
    file2<<"];"<<std::endl;
    file2.close();

    std::ofstream file5;
    std::string file_name5 = "testCoMForce_wrenchDesired3.m";
    file5.open(file_name5);
    file5<<"wrench_desired_lankle_rankle = ["<<std::endl;
    for(unsigned int i = 0; i < wrench_desired.size(); ++i)
        file5<<wrench_desired[i].toString()<<std::endl;
    file5<<"];"<<std::endl;
    file5.close();

    std::ofstream file3;
    std::string file_name3 = "testCoMForce_com3.m";
    file3.open(file_name3);
    file3<<"com = ["<<std::endl;
    for(unsigned int i = 0; i < com.size(); ++i)
        file3<<com[i].toString()<<std::endl;
    file3<<"];"<<std::endl;
    file3.close();

    std::ofstream file4;
    std::string file_name4 = "testCoMForce_com_d3.m";
    file4.open(file_name4);
    file4<<"com_d = ["<<std::endl;
    for(unsigned int i = 0; i < com_d.size(); ++i)
        file4<<com_d[i].toString()<<std::endl;
    file4<<"];"<<std::endl;
    file4.close();

    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

#endif

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
