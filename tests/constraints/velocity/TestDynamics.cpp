#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <numeric>
#include <qpOASES.hpp>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace{

class testDynamicsConstr : public ::testing::Test {
 protected:

  testDynamicsConstr()  :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
      q(coman.iDyn3_model.getNrOfDOFs()),
      q_dot(q.size())
  {
        q.zero();
        q_dot.zero();
  }

  virtual ~testDynamicsConstr() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils coman;
  yarp::sig::Vector q;
  yarp::sig::Vector q_dot;
};

TEST_F(testDynamicsConstr, ID_from_iDynThree) {
    coman.updateiDyn3Model(q, true);

    yarp::sig::Vector tau_g = coman.iDyn3_model.getTorques();
    std::cout<<"tau_g @ q = zeros: ["<<tau_g.toString()<<"]"<<std::endl;
    EXPECT_EQ(tau_g.size(), coman.iDyn3_model.getNrOfDOFs());

    yarp::sig::Matrix M(6+coman.iDyn3_model.getNrOfDOFs(), 6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M);
    EXPECT_DOUBLE_EQ(M.rows(), 6+coman.iDyn3_model.getNrOfDOFs());
    EXPECT_DOUBLE_EQ(M.cols(), 6+coman.iDyn3_model.getNrOfDOFs());
    std::cout<<"M @ q = zeros: ["<<M.toString()<<"]"<<std::endl;

    for(unsigned int i = 0; i < q_dot.size(); ++i)
        q_dot[i] = 0.01;
    coman.updateiDyn3Model(q, q_dot, true);
    yarp::sig::Vector tau_g_C = coman.iDyn3_model.getTorques();

    bool a = false;
    for(unsigned int i = 0; i < tau_g.size(); ++i){
        if(fabs(tau_g[i]-tau_g_C[i])>1e-9){ //at least one is different!
            a = true;
            break;}
    }
    EXPECT_TRUE(a);
    std::cout<<"tau_g @ q = zeros: \n ["<<tau_g.toString()<<"]"<<std::endl;
    std::cout<<"tau_g_C @ q = zeros, dq = 0.01: \n ["<<tau_g_C.toString()<<"]"<<std::endl;

    yarp::sig::Matrix M2(6+coman.iDyn3_model.getNrOfDOFs(), 6+coman.iDyn3_model.getNrOfDOFs());
    coman.iDyn3_model.getFloatingBaseMassMatrix(M2);
    for(unsigned int i = 0; i < M.rows(); ++i)
        for(unsigned int j = 0; j < M.cols(); ++j)
            EXPECT_DOUBLE_EQ(M(i,j), M2(i,j));
}

TEST_F(testDynamicsConstr, testLinkCrawling) {
    std::list<std::string> links_in_contact = coman.getLinksInContact();
    std::cout<<"links in contact: "<<std::endl;
    std::list<std::string>::iterator link;
    for(link = links_in_contact.begin(); link != links_in_contact.end(); link++)
        std::cout<<"    "<<*link<<std::endl;

    std::cout<<std::endl; std::cout<<std::endl;

    std::vector<std::string> ft_links;
    ft_links.push_back("l_ankle");
    ft_links.push_back("r_ankle");
    ft_links.push_back("l_arm_ft");
    ft_links.push_back("r_arm_ft");

    //fake numbers, here we test just a static function
    //OpenSoT::constraints::velocity::Dynamics constr(q,q,q,coman,3,1);
    std::vector<std::string> ft_in_contact;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(ft_links,
                      links_in_contact,
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),2);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[0]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links[1]);


    std::list<std::string> links_in_contact2 = coman.getLinksInContact();
    links_in_contact2.push_back("l_wrist");
    std::cout<<"links in contact2: "<<std::endl;
    for(link = links_in_contact2.begin(); link != links_in_contact2.end(); link++)
        std::cout<<"    "<<*link<<std::endl;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(ft_links,
                      links_in_contact2,
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;
    EXPECT_EQ(ft_in_contact.size(),3);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[0]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links[1]);
    EXPECT_TRUE(ft_in_contact[2] == ft_links[2]);


    for(unsigned int i = 0; i < 4; ++i)
        links_in_contact.pop_front();

    std::cout<<std::endl;
    std::cout<<"links in contact: "<<std::endl;
    for(link = links_in_contact.begin(); link != links_in_contact.end(); link++)
        std::cout<<"    "<<*link<<std::endl;

    OpenSoT::constraints::velocity::Dynamics::crawlLinks(ft_links,
                      links_in_contact,
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),1);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[1]);

    links_in_contact.push_back("r_wrist");

    std::cout<<std::endl;
    std::cout<<"links in contact: "<<std::endl;
    for(link = links_in_contact.begin(); link != links_in_contact.end(); link++)
        std::cout<<"    "<<*link<<std::endl;

    OpenSoT::constraints::velocity::Dynamics::crawlLinks(ft_links,
                      links_in_contact,
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),2);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[1]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links[3]);



    std::vector<std::string> ft_links2;
    ft_links2.push_back("l_ankle");
    ft_links2.push_back("r_ankle");
    ft_links2.push_back("l_wrist");
    ft_links2.push_back("r_wrist");
    iDynUtils coman2("coman",
          std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
          std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    std::list<std::string> links_in_contact3 = coman2.getLinksInContact();
    links_in_contact3.push_back("l_hand_upper_right_link");
    coman2.setLinksInContact(links_in_contact3);
    links_in_contact3 = coman2.getLinksInContact();
    std::cout<<"links in contact: "<<std::endl;
    for(link = links_in_contact3.begin(); link != links_in_contact3.end(); link++)
        std::cout<<"    "<<*link<<std::endl;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(ft_links2,
                      links_in_contact3,
                      coman2, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),3);
    EXPECT_TRUE(ft_in_contact[0] == ft_links2[0]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links2[1]);
    EXPECT_TRUE(ft_in_contact[2] == ft_links2[2]);

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
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

using namespace yarp::sig;
using namespace OpenSoT;
#if OPENSOT_COMPILE_SIMULATION_TESTS

TEST_F(testDynamicsConstr, testFTSensors) {
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);

    yarp::os::Time::delay(4);

    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

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

    yarp::os::Time::delay(10);

    yarp::sig::Vector dq(coman_robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector tau(coman_robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    coman_robot.sense(q, dq, tau);
    RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
    std::map<std::string, yarp::sig::Vector>::iterator it;
    for(it = ft_readings.begin(); it !=  ft_readings.end(); it++)
        std::cout<<"FT Read @ "<<it->first<<" is ["<<it->second.toString()<<"]"<<std::endl;

    std::cout<<"left leg sensed torque = ["<<tau.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;

    coman_robot.idynutils.updateiDyn3Model(q, true);
    yarp::sig::Vector tau_g = coman_robot.idynutils.iDyn3_model.getTorques();

    yarp::sig::Matrix waist_J_l_ft_sensor;
    coman_robot.idynutils.iDyn3_model.getRelativeJacobian(
                coman_robot.idynutils.iDyn3_model.getLinkIndex("l_leg_ft"),
                coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"), waist_J_l_ft_sensor);

    std::cout<<"Leg Jacobian expressed in base: ["<<std::endl;
    for(unsigned int ii = 0; ii < waist_J_l_ft_sensor.rows(); ++ii)
        std::cout<<waist_J_l_ft_sensor.subrow(ii, coman_robot.idynutils.left_leg.joint_numbers[0], 6).toString()<<std::endl;
    std::cout<<"]"<<std::endl;

    yarp::sig::Vector tau_computed = waist_J_l_ft_sensor.transposed()*-1.0*ft_readings["l_leg_ft"];
    std::cout<<"left leg computed torque = ["<<tau_computed.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;
    tau_computed = tau_g + waist_J_l_ft_sensor.transposed()*-1.0*ft_readings["l_leg_ft"];
    std::cout<<"left leg computed torque w/ g = ["<<tau_computed.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;

    std::cout<<"gravity compensation computed torque = ["<<tau_g.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;

    std::cout<<std::endl;
    yarp::sig::Vector error = tau - tau_g - waist_J_l_ft_sensor.transposed()*-1.0*ft_readings["l_leg_ft"];
    std::cout<<"error tau - g - J^t(-f) = ["<<error.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;
    error = tau - waist_J_l_ft_sensor.transposed()*-1.0*ft_readings["l_leg_ft"];
    std::cout<<"error tau - J^t(-f) = ["<<error.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;
    error = tau + tau_g - waist_J_l_ft_sensor.transposed()*-1.0*ft_readings["l_leg_ft"];
    std::cout<<"error tau + g - J^t(-f) = ["<<error.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;
    error = tau + tau_g - waist_J_l_ft_sensor.transposed()*ft_readings["l_leg_ft"];
    std::cout<<"error tau + g - J^t(f) = ["<<error.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;
    error = tau - tau_g - waist_J_l_ft_sensor.transposed()*ft_readings["l_leg_ft"];
    std::cout<<"error tau - g - J^t(f) = ["<<error.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<"]"<<std::endl;


    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

TEST_F(testDynamicsConstr, testConstraint) {

    // Start YARP Server
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman_fixed.world";
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);

    sleep(4);


    double loop_time_average_1 = 0.0;
    double loop_time_average_2 = 0.0;

    double dT = 0.001;
    double joint_velocity_limits = M_PI_2;
    double joint_torque_limit_factor = 0.9;
    double sigma_dynamic_constraint = 0.9;
    double eps = 2e11;
    bool enable_clipping = true;

    std::vector<double> time;

for(unsigned int j = 0; j < 2; ++j){
    if(j == 0)
        std::cout<<RED<<"TEST w/o Dynamic Constr! j = 0"<<DEFAULT<<std::endl;
    else{
        std::cout<<GREEN<<"TEST w/ Dynamic Constr! j = 1"<<DEFAULT<<std::endl;
        if(enable_clipping)
            std::cout<<GREEN<<"Clipping is active"<<std::endl;
        else
            std::cout<<RED<<"Clipping is NOT active"<<std::endl;
    }

    sleep(5);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");


    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

    //Homing
    //In this test no links are in contact with the environment
    coman_robot.idynutils.setLinksInContact(std::list<std::string>());
    coman_robot.idynutils.updateiDyn3Model(q,true);
    coman_robot.setPositionDirectMode();
    coman_robot.move(q);

    //Set Up SoT
    sleep(5);
    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));


    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(joint_velocity_limits, dT,q.size()));


    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));
    // TASKS
    tasks::velocity::Cartesian::Ptr cartesian_task_l_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::l_wrist", q,
                    coman_robot.idynutils,"l_wrist", "Waist"));
    Matrix goal = cartesian_task_l_wrist->getActualPose();
    goal(0,3) += 0.5;
    cartesian_task_l_wrist->setReference(goal);

    tasks::velocity::Cartesian::Ptr cartesian_task_r_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::r_wrist", q,
                    coman_robot.idynutils,"r_wrist", "world"));

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(cartesian_task_l_wrist);
    cartesianTasks.push_back(cartesian_task_r_wrist);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasks,q.size()));

    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

    std::list<tasks::velocity::Cartesian::TaskPtr> jointTasks;
    jointTasks.push_back(postural_task);
    Task<Matrix, Vector>::TaskPtr taskJointAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(jointTasks,q.size()));

    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(taskJointAggregated);

    constraints::velocity::Dynamics::Ptr Dyn;
    if(j == 1)
    {
        yarp::sig::Vector zero(q.size(), 0.0);
        Dyn = constraints::velocity::Dynamics::Ptr(
                new constraints::velocity::Dynamics(q,zero,
                    joint_torque_limit_factor*coman_robot.idynutils.iDyn3_model.getJointTorqueMax(),
                    coman_robot.idynutils, dT,sigma_dynamic_constraint));
        Dyn->setConstraintClipperValue(enable_clipping);
    }

    solvers::QPOases_sot::Ptr sot;
    if(j == 0)
        sot = solvers::QPOases_sot::Ptr(
                new solvers::QPOases_sot(stack_of_tasks, bounds, eps));
    else
        sot = solvers::QPOases_sot::Ptr(
                   new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));

    for(unsigned int ii = 0; ii < stack_of_tasks.size(); ++ii){
        qpOASES::Options opt;
        sot->getOptions(ii, opt);
        opt.setToDefault();
        opt.printLevel = qpOASES::PL_NONE;
        sot->setOptions(ii, opt);
    }


    yarp::sig::Vector dq(q.size(), 0.0);
    std::vector<yarp::sig::Vector> sensed_torque_exp;
    std::vector<yarp::sig::Vector> cartesian_error_exp;
    std::vector<yarp::sig::Vector> computed_velocity_exp;
    int steps = 5000;
    sensed_torque_exp.reserve(steps);
    cartesian_error_exp.reserve(steps);
    computed_velocity_exp.reserve(steps);
    time.reserve(steps);

    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();

        yarp::sig::Vector q_sensed(q.size(), 0.0);
        yarp::sig::Vector dq_sensed(q.size(), 0.0);
        yarp::sig::Vector tau_sensed(q.size(), 0.0);
        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);

        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, true);

        bounds->update(q);
        taskCartesianAggregated->update(q);
        taskJointAggregated->update(q);

        if(j == 1)
            Dyn->update(cat(q,dq/dT));

        if(sot->solve(dq)){
            q += dq;}
        coman_robot.move(q);

        double toc = yarp::os::Time::now();
        time.push_back((toc-tic));

        sensed_torque_exp.push_back(tau_sensed);
        cartesian_error_exp.push_back(cartesian_task_l_wrist->getError());
        computed_velocity_exp.push_back(dq);

        toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
    }

    std::ofstream file1;
    std::string file_name = "testDynamics_torque_exp_"+std::to_string(j)+"_left_arm.m";
    file1.open(file_name);
    file1<<"tau_"<<j<<" = ["<<std::endl;

    std::ofstream file2;
    file_name = "testDynamics_cartesian_error_exp_"+std::to_string(j)+"_left_arm.m";
    file2.open(file_name);
    file2<<"cartesian_error_"<<j<<" = ["<<std::endl;

    std::ofstream file3;
    file_name = "testDynamics_computed_vel_exp_"+std::to_string(j)+"_left_arm.m";
    file3.open(file_name);
    file3<<"computed_vel_"<<j<<" = ["<<std::endl;

    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
        yarp::sig::Vector tau = sensed_torque_exp[i];
        file1<<yarp::math::cat(
                   tau.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                 coman_robot.idynutils.torso.joint_numbers[2]),
                tau.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                              coman_robot.idynutils.left_arm.joint_numbers[6])
                ).toString()<<std::endl;
        file2<<cartesian_error_exp[i].toString()<<std::endl;
        file3<<yarp::math::cat(
                computed_velocity_exp[i].subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                 coman_robot.idynutils.torso.joint_numbers[2]),
                computed_velocity_exp[i].subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                              coman_robot.idynutils.left_arm.joint_numbers[6])
                ).toString()<<std::endl;
    }
    file1<<"];"<<std::endl;
    file1.close();
    file2<<"];"<<std::endl;
    file2.close();
    file3<<"];"<<std::endl;
    file3.close();

    if(j == 1)
    {
        std::ofstream file4;
        file_name = "testDynamics_joint_torque_limits_"+std::to_string(j)+"_left_arm.m";
        file4.open(file_name);
        file4<<"torque_limits"<<j<<" = ["<<std::endl;

        yarp::sig::Vector torque_limits_upper = Dyn->getTorqueLimits();
        for(unsigned int i = 0; i < steps; ++i)
        {
            file4<<yarp::math::cat(torque_limits_upper.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                   coman_robot.idynutils.torso.joint_numbers[2]),
                    torque_limits_upper.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                    coman_robot.idynutils.left_arm.joint_numbers[6])
                    ).toString()<<std::endl;
        }
        file4<<"];"<<std::endl;
    }

    if(j == 1){
        for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
            yarp::sig::Vector t = sensed_torque_exp[i];
            yarp::sig::Vector x = yarp::math::cat(
                               t.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                             coman_robot.idynutils.torso.joint_numbers[2]),
                               t.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
            yarp::sig::Vector xx = yarp::math::cat(
                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                             coman_robot.idynutils.torso.joint_numbers[2]),
                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
            for(unsigned int jj = 0; jj < x.size(); ++jj)
                EXPECT_LE(fabs(x[jj]), 0.9*xx[jj])<<"@joint "<<jj;
        }
    }

    if(j == 0){
        for(unsigned int i = 0; i < time.size(); ++i)
            loop_time_average_1 += time[i];
        loop_time_average_1 = loop_time_average_1/double(time.size());
    }
    else
        for(unsigned int i = 0; i < time.size(); ++i)
            loop_time_average_2 += time[i];
        loop_time_average_2 = loop_time_average_2/double(time.size());

    time.clear();

}

    std::ofstream file6;
    std::string file_name = "testDynamicsConstr_testConstraint.m";
    file6.open(file_name);
    file6<<"LOOP_TIME_AVERAGE_NO_Dynamic_Constr= "<<loop_time_average_1<<std::endl;
    file6<<"LOOP_TIME_AVERAGE_Dynamic_Constr= "<<loop_time_average_2<<std::endl;
    file6<<"dT= "<<dT<<std::endl;
    file6<<"joint_velocity_limit= "<<joint_velocity_limits<<std::endl;
    file6<<"joint_torque_limit_factor= "<<joint_torque_limit_factor<<std::endl;
    file6<<"sigma_dynamic_constraint= "<<sigma_dynamic_constraint<<std::endl;
    file6<<"eps_regularization= "<<eps<<std::endl;
    file6.close();


    std::cout<<"LOOP TIME AVERAGE w/o Dynamic Constr: "<<loop_time_average_1<<" [s]"<<std::endl;
    std::cout<<"LOOP TIME AVERAGE w Dynamic Constr: "<<loop_time_average_2<<" [s]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"TEST PARAMETERS:"<<std::endl;
    std::cout<<"dT: "<<dT<<" [s]"<<std::endl;
    std::cout<<"joint_velocity_limit: "<<joint_velocity_limits<<" [rad/sec]"<<std::endl;
    std::cout<<"joint_torque_limit_factor: "<<joint_torque_limit_factor<<std::endl;
    std::cout<<"sigma_dynamic_constraint: "<<sigma_dynamic_constraint<<std::endl;
    std::cout<<"eps regularization: "<<eps<<std::endl;


    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}


void linear_trj(const yarp::sig::Matrix& A, const yarp::sig::Matrix& B,
                             const double t, const double p,
                             yarp::sig::Matrix& pose_trj, yarp::sig::Vector& vel_trj)
{
    pose_trj = A;

    for(unsigned int i = 0; i < 3; ++i){
        pose_trj(i,3) = A(i,3) + std::sin(t*M_PI/(2.*p))*(B(i,3)-A(i,3));
        vel_trj(i) = M_PI/(2.*p)*std::cos(t*M_PI/(2.*p))*(B(i,3)-A(i,3));
    }
}

TEST_F(testDynamicsConstr, testConstraintWithTrj) {

    // Start YARP Server
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman_fixed.world";
    if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);

    sleep(4);


    double loop_time_average_1 = 0.0;
    double loop_time_average_2 = 0.0;

    double T_trj = 3.0;
    double t_trj = 0.0;

    double dT = 0.001;
    double joint_velocity_limits = M_PI_2;
    double joint_torque_limit_factor = 0.9;
    double sigma_dynamic_constraint = 0.9;
    double eps = 2e11;
    bool enable_clipping = true;

    std::vector<double> time;

    Matrix start(4,4);
    Matrix goal(4,4);

for(unsigned int j = 0; j < 2; ++j){
    t_trj = 0.0;
    if(j == 0)
        std::cout<<RED<<"TEST w/o Dynamic Constr! j = 0"<<DEFAULT<<std::endl;
    else{
        std::cout<<GREEN<<"TEST w/ Dynamic Constr! j = 1"<<DEFAULT<<std::endl;
        if(enable_clipping)
            std::cout<<GREEN<<"Clipping is enabled"<<DEFAULT<<std::endl;
        else
            std::cout<<RED<<"Clipping is NOT enabled"<<DEFAULT<<std::endl;
    }

    sleep(5);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");


    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

    //Homing
    //In this test no links are in contact with the environment
    coman_robot.idynutils.setLinksInContact(std::list<std::string>());
    coman_robot.idynutils.updateiDyn3Model(q,true);
    coman_robot.setPositionDirectMode();
    coman_robot.move(q);

    //Set Up SoT
    sleep(5);
    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));


    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(joint_velocity_limits, dT,q.size()));


    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));
    // TASKS
    tasks::velocity::Cartesian::Ptr cartesian_task_l_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::l_wrist", q,
                    coman_robot.idynutils,"l_wrist", "Waist"));

    start = cartesian_task_l_wrist->getActualPose();
    goal = start;
    goal(0,3) += 0.5;


    tasks::velocity::Cartesian::Ptr cartesian_task_r_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::r_wrist", q,
                    coman_robot.idynutils,"r_wrist", "world"));

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(cartesian_task_l_wrist);
    cartesianTasks.push_back(cartesian_task_r_wrist);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasks,q.size()));

    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

    std::list<tasks::velocity::Cartesian::TaskPtr> jointTasks;
    jointTasks.push_back(postural_task);
    Task<Matrix, Vector>::TaskPtr taskJointAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(jointTasks,q.size()));

    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(taskJointAggregated);

    constraints::velocity::Dynamics::Ptr Dyn;
    if(j == 1)
    {
        yarp::sig::Vector zero(q.size(), 0.0);
        Dyn = constraints::velocity::Dynamics::Ptr(
                new constraints::velocity::Dynamics(q,zero,
                    joint_torque_limit_factor*coman_robot.idynutils.iDyn3_model.getJointTorqueMax(),
                    coman_robot.idynutils, dT,sigma_dynamic_constraint));
        Dyn->setConstraintClipperValue(enable_clipping);
    }

    solvers::QPOases_sot::Ptr sot;
    if(j == 0)
        sot = solvers::QPOases_sot::Ptr(
                new solvers::QPOases_sot(stack_of_tasks, bounds, eps));
    else
        sot = solvers::QPOases_sot::Ptr(
                   new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));

    for(unsigned int ii = 0; ii < stack_of_tasks.size(); ++ii){
        qpOASES::Options opt;
        sot->getOptions(ii, opt);
        opt.setToDefault();
        opt.printLevel = qpOASES::PL_NONE;
        sot->setOptions(ii, opt);
    }


    yarp::sig::Vector dq(q.size(), 0.0);
    std::vector<yarp::sig::Vector> sensed_torque_exp;
    std::vector<yarp::sig::Vector> cartesian_error_exp;
    std::vector<yarp::sig::Vector> computed_velocity_exp;
    int steps = 5000;
    sensed_torque_exp.reserve(steps);
    cartesian_error_exp.reserve(steps);
    computed_velocity_exp.reserve(steps);
    time.reserve(steps);

    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();

        yarp::sig::Vector q_sensed(q.size(), 0.0);
        yarp::sig::Vector dq_sensed(q.size(), 0.0);
        yarp::sig::Vector tau_sensed(q.size(), 0.0);
        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);

        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, true);

        if(t_trj <= T_trj){
            yarp::sig::Matrix p(4,4);
            yarp::sig::Vector v(6, 0.0);
            linear_trj(start, goal, t_trj, T_trj, p, v);
            cartesian_task_l_wrist->setReference(p, v);
        }

        bounds->update(q);
        taskCartesianAggregated->update(q);
        taskJointAggregated->update(q);

        if(j == 1)
            Dyn->update(cat(q,dq/dT));

        if(sot->solve(dq)){
            q += dq;}
        coman_robot.move(q);

        double toc = yarp::os::Time::now();
        time.push_back((toc-tic));

        sensed_torque_exp.push_back(tau_sensed);
        cartesian_error_exp.push_back(cartesian_task_l_wrist->getError());
        computed_velocity_exp.push_back(dq);

        toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));

        if(t_trj <= T_trj)
            t_trj += dT;
    }

    std::ofstream file1;
    std::string file_name = "testDynamicsTrj_torque_exp_"+std::to_string(j)+"_left_arm.m";
    file1.open(file_name);
    file1<<"tau_"<<j<<" = ["<<std::endl;

    std::ofstream file2;
    file_name = "testDynamicsTrj_cartesian_error_exp_"+std::to_string(j)+"_left_arm.m";
    file2.open(file_name);
    file2<<"cartesian_error_"<<j<<" = ["<<std::endl;

    std::ofstream file3;
    file_name = "testDynamicsTrj_computed_vel_exp_"+std::to_string(j)+"_left_arm.m";
    file3.open(file_name);
    file3<<"computed_vel_"<<j<<" = ["<<std::endl;

    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
        yarp::sig::Vector tau = sensed_torque_exp[i];
        file1<<yarp::math::cat(
                   tau.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                 coman_robot.idynutils.torso.joint_numbers[2]),
                tau.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                              coman_robot.idynutils.left_arm.joint_numbers[6])
                ).toString()<<std::endl;
        file2<<cartesian_error_exp[i].toString()<<std::endl;
        file3<<yarp::math::cat(
                computed_velocity_exp[i].subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                 coman_robot.idynutils.torso.joint_numbers[2]),
                computed_velocity_exp[i].subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                              coman_robot.idynutils.left_arm.joint_numbers[6])
                ).toString()<<std::endl;
    }
    file1<<"];"<<std::endl;
    file1.close();
    file2<<"];"<<std::endl;
    file2.close();
    file3<<"];"<<std::endl;
    file3.close();

    if(j == 1)
    {
        std::ofstream file4;
        file_name = "testDynamicsTrj_joint_torque_limits_"+std::to_string(j)+"_left_arm.m";
        file4.open(file_name);
        file4<<"torque_limits"<<j<<" = ["<<std::endl;

        yarp::sig::Vector torque_limits_upper = Dyn->getTorqueLimits();
        for(unsigned int i = 0; i < steps; ++i)
        {
            file4<<yarp::math::cat(torque_limits_upper.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                   coman_robot.idynutils.torso.joint_numbers[2]),
                    torque_limits_upper.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                    coman_robot.idynutils.left_arm.joint_numbers[6])
                    ).toString()<<std::endl;
        }
        file4<<"];"<<std::endl;
    }

    if(j == 1){
        for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
            yarp::sig::Vector t = sensed_torque_exp[i];
            yarp::sig::Vector x = yarp::math::cat(
                               t.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                             coman_robot.idynutils.torso.joint_numbers[2]),
                               t.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
            yarp::sig::Vector xx = yarp::math::cat(
                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.torso.joint_numbers[0],
                                             coman_robot.idynutils.torso.joint_numbers[2]),
                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
            for(unsigned int jj = 0; jj < x.size(); ++jj)
                EXPECT_LE(fabs(x[jj]), 0.9*xx[jj])<<"@joint "<<jj;
        }
    }

    if(j == 0){
        for(unsigned int i = 0; i < time.size(); ++i)
            loop_time_average_1 += time[i];
        loop_time_average_1 = loop_time_average_1/double(time.size());
    }
    else
        for(unsigned int i = 0; i < time.size(); ++i)
            loop_time_average_2 += time[i];
        loop_time_average_2 = loop_time_average_2/double(time.size());

    time.clear();

}

    std::ofstream file6;
    std::string file_name = "testDynamicsConstr_testConstraintTrj.m";
    file6.open(file_name);
    file6<<"LOOP_TIME_AVERAGE_NO_Dynamic_Constr= "<<loop_time_average_1<<std::endl;
    file6<<"LOOP_TIME_AVERAGE_Dynamic_Constr= "<<loop_time_average_2<<std::endl;
    file6<<"dT= "<<dT<<std::endl;
    file6<<"joint_velocity_limit= "<<joint_velocity_limits<<std::endl;
    file6<<"joint_torque_limit_factor= "<<joint_torque_limit_factor<<std::endl;
    file6<<"sigma_dynamic_constraint= "<<sigma_dynamic_constraint<<std::endl;
    file6<<"eps_regularization= "<<eps<<std::endl;
    file6.close();


    std::cout<<"LOOP TIME AVERAGE w/o Dynamic Constr: "<<loop_time_average_1<<" [s]"<<std::endl;
    std::cout<<"LOOP TIME AVERAGE w Dynamic Constr: "<<loop_time_average_2<<" [s]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"TEST PARAMETERS:"<<std::endl;
    std::cout<<"dT: "<<dT<<" [s]"<<std::endl;
    std::cout<<"joint_velocity_limit: "<<joint_velocity_limits<<" [rad/sec]"<<std::endl;
    std::cout<<"joint_torque_limit_factor: "<<joint_torque_limit_factor<<std::endl;
    std::cout<<"sigma_dynamic_constraint: "<<sigma_dynamic_constraint<<std::endl;
    std::cout<<"eps regularization: "<<eps<<std::endl;


    std::ofstream file7;
    file_name = "testDynamicsTrj_CartesianTrj.m";
    file7.open(file_name);
    file7<<"T_"<<" = ["<<std::endl;

    std::ofstream file8;
    file_name = "testDynamicsTrj_CartesianVelTrj.m";
    file8.open(file_name);
    file8<<"v_"<<" = ["<<std::endl;
    t_trj = 0.0;
    for(unsigned int i = 0; i < T_trj*int(1/dT); ++i)
    {
        yarp::sig::Matrix T(4,4);
        yarp::sig::Vector v(6,0.0);
        linear_trj(start, goal, t_trj, T_trj, T, v);

        KDL::Frame T_KDL;
        cartesian_utils::fromYARPMatrixtoKDLFrame(T, T_KDL);

        double roll, pitch, yaw;
        T_KDL.M.GetRPY(roll, pitch, yaw);
        file7<<T_KDL.p.x()<<" "<<T_KDL.p.y()<<" "<<T_KDL.p.z()<<" "<<roll<<" "<<pitch<<" "<<yaw<<std::endl;
        file8<<v.toString()<<std::endl;

        t_trj += dT;
    }
    file7<<"];"<<std::endl;
    file8<<"];"<<std::endl;
    file7.close();
    file8.close();



    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

TEST_F(testDynamicsConstr, testConstraintWithContacts) {

    for(unsigned int j = 1; j < 2; ++j){
//    if(j == 1){
//        sleep(5);
//        bool success = tests_utils::stopGazebo();
//        if(success)
//            std::cout<<"GAZEBO KILLED"<<std::endl;
//        sleep(5);
//        success = tests_utils::stopYarpServer();
//        if(success)
//            std::cout<<"yarpserver KILLED"<<std::endl;
//        sleep(5);
//    }

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
    sleep(10);


    double dT = 0.001;
    double joint_velocity_limits = 0.6;
    double joint_torque_limit_factor = 0.4;
    double sigma_dynamic_constraint = 0.5;
    double eps = 2e11;
    bool enable_clipping = false;


    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));


    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(joint_velocity_limits, dT,q.size()));


    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));
    // TASKS
    std::vector<bool> active_joints;

    tasks::velocity::Cartesian::Ptr cartesian_task_l_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::l_wrist", q,
                    coman_robot.idynutils,"l_wrist", "world"));
    Matrix intial_pose_l_wrist = cartesian_task_l_wrist->getActualPose();

    tasks::velocity::Cartesian::Ptr cartesian_task_r_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::r_wrist", q,
                    coman_robot.idynutils,"r_wrist", "world"));
    Matrix intial_pose_r_wrist = cartesian_task_r_wrist->getActualPose();

    tasks::velocity::Cartesian::Ptr cartesian_task_waist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::waist", q,
                    coman_robot.idynutils,"Waist", "world"));
    yarp::sig::Matrix WWW(6,6); WWW.eye();
    WWW(0,0) = 0.0; WWW(1,1) = 0.0; WWW(2,2) = 0.0;
    cartesian_task_waist->setWeight(WWW);

    active_joints = cartesian_task_l_wrist->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.torso.getNrOfDOFs(); ++i)
        active_joints[coman_robot.idynutils.torso.joint_numbers[i]] = false;
    cartesian_task_l_wrist->setActiveJointsMask(active_joints);
    cartesian_task_r_wrist->setActiveJointsMask(active_joints);

    tasks::velocity::CoM::Ptr com_task(new tasks::velocity::CoM(q, coman_robot.idynutils));
    yarp::sig::Matrix WW(3,3); WW.eye();
    WW(2,2) = 0.0;
    com_task->setWeight(WW);


    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));
    tasks::velocity::MinimizeAcceleration::Ptr min_acc_task=
            tasks::velocity::MinimizeAcceleration::Ptr(new tasks::velocity::MinimizeAcceleration(q));
    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> jointTasks;
    jointTasks.push_back(postural_task);
    jointTasks.push_back(min_acc_task);
     OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskJointAggregated =
             OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(jointTasks,q.size()));

    tasks::velocity::Cartesian::Ptr right_foot(new tasks::velocity::Cartesian("cartesian::r_foot",q,
        coman_robot.idynutils, coman_robot.idynutils.right_leg.end_effector_name, "world"));
    tasks::velocity::Cartesian::Ptr torso(new tasks::velocity::Cartesian("cartesian::torso",q,
        coman_robot.idynutils, "torso", "world"));
    yarp::sig::Matrix W(6,6); W.eye();
    W(0,0) = 0.0; W(1,1) = 0.0; W(2,2) = 0.0;
    torso->setWeight(W);
    active_joints = torso->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.left_leg.getNrOfDOFs(); ++i)
        active_joints[coman_robot.idynutils.left_leg.joint_numbers[i]] = false;
    torso->setActiveJointsMask(active_joints);

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasksHighest;
    cartesianTasksHighest.push_back(right_foot);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregatedHighest =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasksHighest,q.size()));

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(cartesian_task_l_wrist);
    cartesianTasks.push_back(cartesian_task_r_wrist);
    cartesianTasks.push_back(cartesian_task_waist);
    cartesianTasks.push_back(torso);
    cartesianTasks.push_back(com_task);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasks,q.size()));


    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregatedHighest);
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(taskJointAggregated);


    yarp::sig::Vector tau_max = coman_robot.idynutils.iDyn3_model.getJointTorqueMax();
    for(unsigned int i = 0; i < coman_robot.left_leg.getNumberOfJoints(); ++i){
        tau_max[coman_robot.idynutils.left_leg.joint_numbers[i]] *= joint_torque_limit_factor;
        tau_max[coman_robot.idynutils.right_leg.joint_numbers[i]] *= joint_torque_limit_factor;}
    yarp::sig::Vector zero(q.size(), 0.0);
    constraints::velocity::Dynamics::Ptr Dyn = constraints::velocity::Dynamics::Ptr(
                new constraints::velocity::Dynamics(q,zero,
                    tau_max,
                    coman_robot.idynutils, dT,sigma_dynamic_constraint));
    Dyn->setConstraintClipperValue(enable_clipping);


    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
    if(j == 1)
        sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));
    else
        sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, eps));


    yarp::sig::Vector dq(q.size(), 0.0);
    std::vector<yarp::sig::Vector> sensed_torque_exp;
    std::vector<yarp::sig::Vector> cartesian_error_exp;
    std::vector<yarp::sig::Vector> computed_velocity_exp;
    int steps = int(2.*M_PI*1000);//2.
    sensed_torque_exp.reserve(steps);
    cartesian_error_exp.reserve(steps);
    computed_velocity_exp.reserve(steps);
    for(unsigned int i = 0; i < steps; ++i)
    {

        //yarp::sig::Vector dq_m = coman_robot.senseVelocity();

        double tic = yarp::os::Time::now();
        RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second += (ft_readings[_ft_measurements[i].first]-_ft_measurements[i].second)*0.9;

        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second = -1.0*_ft_measurements[i].second;
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second = -1.0*_ft_measurements[i].second;

//        yarp::sig::Matrix M(6+coman_robot.getNumberOfJoints(), 6+coman_robot.getNumberOfJoints());
//        coman_robot.idynutils.iDyn3_model.getFloatingBaseMassMatrix(M);
//        M.removeCols(0,6); M.removeRows(0,6);
//        postural_task->setWeight(M);

        if(i <= M_PI*1000){
            Matrix goal_r_wrist = intial_pose_r_wrist;
            goal_r_wrist(2,3) += (-0.18)*std::sin((i+M_PI)/1000.0);
            cartesian_task_r_wrist->setReference(goal_r_wrist);

            Matrix goal_l_wrist = intial_pose_l_wrist;
            goal_l_wrist(2,3) += (-0.18)*std::sin((i+M_PI)/1000.0);
            cartesian_task_l_wrist->setReference(goal_l_wrist);
        }

        bounds->update(q);
        taskCartesianAggregated->update(q);
        taskCartesianAggregatedHighest->update(q);
        taskJointAggregated->update(q);
        Dyn->update(cat(q,dq/dT));

        cartesian_error_exp.push_back(yarp::math::cat(
                                          cartesian_task_l_wrist->getError(),
                                          cartesian_task_r_wrist->getError()));



        if(sot->solve(dq)){
            computed_velocity_exp.push_back(dq);
            q += dq;}
        coman_robot.move(q);


        yarp::sig::Vector q_sensed(q.size(), 0.0);
        yarp::sig::Vector dq_sensed(q.size(), 0.0);
        yarp::sig::Vector tau_sensed(q.size(), 0.0);
        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);
        sensed_torque_exp.push_back(tau_sensed);

        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
    }

    std::ofstream file1;
    std::string file_name = "testDynamics_max_torques_torso"+std::to_string(j)+".m";
    file1.open(file_name);
    file1<<"tau_max_torso"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file11;
    file_name = "testDynamics_max_torques_left_arm"+std::to_string(j)+".m";
    file11.open(file_name);
    file11<<"tau_max_left_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file12;
    file_name = "testDynamics_max_torques_right_arm"+std::to_string(j)+".m";
    file12.open(file_name);
    file12<<"tau_max_right_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file13;
    file_name = "testDynamics_max_torques_left_leg"+std::to_string(j)+".m";
    file13.open(file_name);
    file13<<"tau_max_left_leg"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file14;
    file_name = "testDynamics_max_torques_right_leg"+std::to_string(j)+".m";
    file14.open(file_name);
    file14<<"tau_max_right_leg"+std::to_string(j)+" = ["<<std::endl;

    yarp::sig::Vector trq_limits = Dyn->getTorqueLimits();
        file1<<trq_limits.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                coman_robot.idynutils.torso.joint_numbers[2]).toString()<<std::endl;
        file11<<trq_limits.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                coman_robot.idynutils.left_arm.joint_numbers[6]).toString()<<std::endl;
        file12<<trq_limits.subVector(coman_robot.idynutils.right_arm.joint_numbers[0],
                coman_robot.idynutils.right_arm.joint_numbers[6]).toString()<<std::endl;
        file13<<trq_limits.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
                coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<std::endl;
        file14<<trq_limits.subVector(coman_robot.idynutils.right_leg.joint_numbers[0],
                coman_robot.idynutils.right_leg.joint_numbers[5]).toString()<<std::endl;


    file1<<"];"<<std::endl;
    file1.close();
    file11<<"];"<<std::endl;
    file11.close();
    file12<<"];"<<std::endl;
    file12.close();
    file13<<"];"<<std::endl;
    file13.close();
    file14<<"];"<<std::endl;
    file14.close();

    std::ofstream file2;
    file_name = "testDynamics_cartesian_error_legsINcontacts_left_right_arm"+std::to_string(j)+".m";
    file2.open(file_name);
    file2<<"cartesian_error"+std::to_string(j)+" = ["<<std::endl;


    std::ofstream file31;
    file_name = "testDynamics_computed_vel_torso"+std::to_string(j)+".m";
    file31.open(file_name);
    file31<<"computed_vel_torso"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file32;
    file_name = "testDynamics_computed_vel_left_arm"+std::to_string(j)+".m";
    file32.open(file_name);
    file32<<"computed_vel_left_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file33;
    file_name = "testDynamics_computed_vel_right_arm"+std::to_string(j)+".m";
    file33.open(file_name);
    file33<<"computed_vel_right_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file34;
    file_name = "testDynamics_computed_vel_left_leg"+std::to_string(j)+".m";
    file34.open(file_name);
    file34<<"computed_vel_left_leg"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file35;
    file_name = "testDynamics_computed_vel_right_leg"+std::to_string(j)+".m";
    file35.open(file_name);
    file35<<"computed_vel_right_leg"+std::to_string(j)+" = ["<<std::endl;


    std::ofstream file4;
    file_name = "testDynamics_torque_torso"+std::to_string(j)+".m";
    file4.open(file_name);
    file4<<"tau_torso"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file41;
    file_name = "testDynamics_torque_left_arm"+std::to_string(j)+".m";
    file41.open(file_name);
    file41<<"tau_left_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file42;
    file_name = "testDynamics_torque_right_arm"+std::to_string(j)+".m";
    file42.open(file_name);
    file42<<"tau_right_arm"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file43;
    file_name = "testDynamics_torque_left_leg"+std::to_string(j)+".m";
    file43.open(file_name);
    file43<<"tau_left_leg"+std::to_string(j)+" = ["<<std::endl;

    std::ofstream file44;
    file_name = "testDynamics_torque_right_leg"+std::to_string(j)+".m";
    file44.open(file_name);
    file44<<"tau_right_leg"+std::to_string(j)+" = ["<<std::endl;

    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
        file2<<cartesian_error_exp[i].toString()<<std::endl;

        yarp::sig::Vector tau = sensed_torque_exp[i];
        file4<<tau.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                coman_robot.idynutils.torso.joint_numbers[2]).toString()<<std::endl;
        file41<<tau.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                coman_robot.idynutils.left_arm.joint_numbers[6]).toString()<<std::endl;
        file42<<tau.subVector(coman_robot.idynutils.right_arm.joint_numbers[0],
                coman_robot.idynutils.right_arm.joint_numbers[6]).toString()<<std::endl;
        file43<<tau.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
                coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<std::endl;
        file44<<tau.subVector(coman_robot.idynutils.right_leg.joint_numbers[0],
                coman_robot.idynutils.right_leg.joint_numbers[5]).toString()<<std::endl;

        yarp::sig::Vector vel = computed_velocity_exp[i];
        file31<<vel.subVector(coman_robot.idynutils.torso.joint_numbers[0],
                coman_robot.idynutils.torso.joint_numbers[2]).toString()<<std::endl;
        file32<<vel.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
                coman_robot.idynutils.left_arm.joint_numbers[6]).toString()<<std::endl;
        file33<<vel.subVector(coman_robot.idynutils.right_arm.joint_numbers[0],
                coman_robot.idynutils.right_arm.joint_numbers[6]).toString()<<std::endl;
        file34<<vel.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
                coman_robot.idynutils.left_arm.joint_numbers[5]).toString()<<std::endl;
        file35<<vel.subVector(coman_robot.idynutils.right_leg.joint_numbers[0],
                coman_robot.idynutils.right_arm.joint_numbers[5]).toString()<<std::endl;

    }


    file31<<"];"<<std::endl;
    file31.close();
    file32<<"];"<<std::endl;
    file32.close();
    file33<<"];"<<std::endl;
    file33.close();
    file34<<"];"<<std::endl;
    file34.close();
    file35<<"];"<<std::endl;
    file35.close();

    file4<<"];"<<std::endl;
    file4.close();
    file41<<"];"<<std::endl;
    file41.close();
    file42<<"];"<<std::endl;
    file42.close();
    file43<<"];"<<std::endl;
    file43.close();
    file44<<"];"<<std::endl;
    file44.close();

    file2<<"];"<<std::endl;
    file2.close();

    std::ofstream file6;
    file_name = "testDynamicsConstr_testConstraintWithContacts.m";
    file6.open(file_name);
    file6<<"dT= "<<dT<<std::endl;
    file6<<"joint_velocity_limit= "<<joint_velocity_limits<<std::endl;
    file6<<"joint_torque_limit_factor= "<<joint_torque_limit_factor<<std::endl;
    file6<<"sigma_dynamic_constraint= "<<sigma_dynamic_constraint<<std::endl;
    file6<<"eps_regularization= "<<eps<<std::endl;
    file6.close();


    std::cout<<std::endl;
    std::cout<<"TEST PARAMETERS:"<<std::endl;
    std::cout<<"dT: "<<dT<<" [s]"<<std::endl;
    std::cout<<"joint_velocity_limit: "<<joint_velocity_limits<<" [rad/sec]"<<std::endl;
    std::cout<<"joint_torque_limit_factor: "<<joint_torque_limit_factor<<std::endl;
    std::cout<<"sigma_dynamic_constraint: "<<sigma_dynamic_constraint<<std::endl;
    std::cout<<"eps regularization: "<<eps<<std::endl;


    if(j == 1){
        for(unsigned int i = 10; i < sensed_torque_exp.size(); ++i){
            yarp::sig::Vector tau = sensed_torque_exp[i];
            for(unsigned int jj = 0; jj < coman_robot.idynutils.left_leg.joint_numbers.size(); ++jj){
                EXPECT_LE(fabs(tau[coman_robot.idynutils.left_leg.joint_numbers[jj]]),
                        tau_max[coman_robot.idynutils.left_leg.joint_numbers[jj]]*1.0)<<"@joint "<<coman_robot.idynutils.left_leg.joint_numbers[jj];
                EXPECT_LE(fabs(tau[coman_robot.idynutils.right_leg.joint_numbers[jj]]),
                        tau_max[coman_robot.idynutils.right_leg.joint_numbers[jj]]*1.0)<<"@joint "<<coman_robot.idynutils.right_leg.joint_numbers[jj];
            }
        }
    }

    }
    sleep(5);
    bool success = tests_utils::stopGazebo();
    if(success)
        std::cout<<"GAZEBO KILLED"<<std::endl;
    sleep(5);
    success = tests_utils::stopYarpServer();
    if(success)
        std::cout<<"yarpserver KILLED"<<std::endl;
    sleep(5);
}
#endif

TEST_F(testDynamicsConstr, DISABLED_testConstraintWithContacts_externalForces) {

    // Applied external force is: base_link 0 0 -200 0 0 0 1

    // Start YARP Server
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    tests_utils::startGazebo(world_path);

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
    sleep(10);


    double dT = 0.001;
    double joint_vel_limits = M_PI;
    double dyn_constr_bound_scaling = 1.0;
    bool   dyn_constr_clip = false;
    double torque_scaling_factor = 0.9;
    double eps = 1e12;
    double ft_filter = 0.75;

    bool enable_dyn_constraint = true;


    if(enable_dyn_constraint){
        std::cout<<GREEN<<"Dyn Constraint is active!!"<<DEFAULT<<std::endl;
        if(dyn_constr_bound_scaling)
            std::cout<<RED<<"Clipping is active!"<<DEFAULT<<std::endl;
        else
            std::cout<<GREEN<<"Clipping is NOT active!"<<DEFAULT<<std::endl;
    }else
        std::cout<<RED<<"Dyn Constraint is NOT active"<<DEFAULT<<std::endl;

    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(joint_vel_limits, dT,q.size()));


    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));
    // TASKS
    std::vector<bool> active_joints;

    tasks::velocity::Cartesian::Ptr cartesian_task_l_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::l_wrist", q,
                    coman_robot.idynutils,"l_wrist", "world"));

    tasks::velocity::Cartesian::Ptr cartesian_task_r_wrist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::r_wrist", q,
                    coman_robot.idynutils,"r_wrist", "world"));

    tasks::velocity::Cartesian::Ptr cartesian_task_waist=
            tasks::velocity::Cartesian::Ptr(
                new tasks::velocity::Cartesian("cartesian::waist", q,
                    coman_robot.idynutils,"Waist", "world"));


    active_joints = cartesian_task_l_wrist->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.torso.getNrOfDOFs(); ++i)
        active_joints[coman_robot.idynutils.torso.joint_numbers[i]] = false;
    cartesian_task_l_wrist->setActiveJointsMask(active_joints);
    cartesian_task_r_wrist->setActiveJointsMask(active_joints);

    tasks::velocity::CoM::Ptr com_task(new tasks::velocity::CoM(q, coman_robot.idynutils));
    yarp::sig::Matrix WW(3,3); WW.eye();
    WW(2,2) = 0.0;
    com_task->setWeight(WW);


    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

    tasks::velocity::MinimizeAcceleration::Ptr min_acc_task=
            tasks::velocity::MinimizeAcceleration::Ptr(new tasks::velocity::MinimizeAcceleration(q));
    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> jointTasks;
    jointTasks.push_back(postural_task);
    jointTasks.push_back(min_acc_task);
     OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskJointAggregated =
             OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(jointTasks,q.size()));

    tasks::velocity::Cartesian::Ptr right_foot(new tasks::velocity::Cartesian("cartesian::r_foot",q,
        coman_robot.idynutils, coman_robot.idynutils.right_leg.end_effector_name, "world"));
    tasks::velocity::Cartesian::Ptr torso(new tasks::velocity::Cartesian("cartesian::torso",q,
        coman_robot.idynutils, "torso", "world"));
    yarp::sig::Matrix W(6,6); W.eye();
    W(0,0) = 0.0; W(1,1) = 0.0; W(2,2) = 0.0;
    torso->setWeight(W);
    active_joints = torso->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.left_leg.getNrOfDOFs(); ++i)
        active_joints[coman_robot.idynutils.left_leg.joint_numbers[i]] = false;
    torso->setActiveJointsMask(active_joints);

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasksHighest;
    cartesianTasksHighest.push_back(right_foot);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregatedHighest =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasksHighest,q.size()));

    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(cartesian_task_l_wrist);
    cartesianTasks.push_back(cartesian_task_r_wrist);
    cartesianTasks.push_back(cartesian_task_waist);
    cartesianTasks.push_back(torso);
    cartesianTasks.push_back(com_task);
    Task<Matrix, Vector>::TaskPtr taskCartesianAggregated =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(cartesianTasks,q.size()));


    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregatedHighest);
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(taskJointAggregated);


    yarp::sig::Vector tau_max = coman_robot.idynutils.iDyn3_model.getJointTorqueMax();
    for(unsigned int i = 0; i < coman_robot.left_leg.getNumberOfJoints(); ++i){
        tau_max[coman_robot.idynutils.left_leg.joint_numbers[i]] *= torque_scaling_factor;
        tau_max[coman_robot.idynutils.right_leg.joint_numbers[i]] *= torque_scaling_factor;}
    yarp::sig::Vector zero(q.size(), 0.0);
    constraints::velocity::Dynamics::Ptr Dyn = constraints::velocity::Dynamics::Ptr(
                new constraints::velocity::Dynamics(q,zero,
                    tau_max,
                    coman_robot.idynutils, dT,dyn_constr_bound_scaling));
    Dyn->setConstraintClipperValue(dyn_constr_clip);


    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
    if(enable_dyn_constraint)
        sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));
    else
        sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, eps));



    yarp::sig::Vector dq(q.size(), 0.0);
    std::vector<yarp::sig::Vector> sensed_torque_exp;
    std::vector<yarp::sig::Vector> com_cartesian_error_exp;
    std::vector<yarp::sig::Vector> computed_velocity_exp;
    std::vector<yarp::sig::Vector> filtered_ft_left_right;

    double t = 0.0;
    while(1)
    {
        double tic = yarp::os::Time::now();
        RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second += (ft_readings[_ft_measurements[i].first]-_ft_measurements[i].second)*ft_filter;

        yarp::sig::Vector q_sensed(q.size(), 0.0);
        yarp::sig::Vector dq_sensed(q.size(), 0.0);
        yarp::sig::Vector tau_sensed(q.size(), 0.0);
        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);

        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second = -1.0*_ft_measurements[i].second;
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second = -1.0*_ft_measurements[i].second;

        bounds->update(q);
        taskCartesianAggregated->update(q);
        taskCartesianAggregatedHighest->update(q);
        taskJointAggregated->update(q);
        Dyn->update(cat(q,dq/dT));


        if(sot->solve(dq))
            q += dq;
        else
            std::cout<<RED<<"SOLVER ERROR"<<DEFAULT<<std::endl;
        coman_robot.move(q);

        double toc = yarp::os::Time::now();


        sensed_torque_exp.push_back(tau_sensed);
        com_cartesian_error_exp.push_back((com_task->getError()).subVector(0,1));
        computed_velocity_exp.push_back(dq);
        filtered_ft_left_right.push_back(yarp::math::cat(
            _ft_measurements[1].second, _ft_measurements[3].second));

        toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));

        if(tests_utils::_kbhit())
        {
            std::cout<<GREEN<<"USER PRESS A BUTTON, EXITINIG..."<<DEFAULT<<std::endl;
            break;
        }

        t += dT;
        if(t >= 5.0){
            std::cout<<"...running..."<<std::endl;
            t = 0.0;}
    }

    std::ofstream file1;
    std::string file_name = "testDynamics_torque_all.m";
    file1.open(file_name.c_str());
    file1<<"tau = ["<<std::endl;

    std::ofstream file2;
    file_name = "testDynamics_cartesian_error_legsINcontacts_com.m";
    file2.open(file_name.c_str());
    file2<<"cartesian_error = ["<<std::endl;

    std::ofstream file3;
    file_name = "testDynamics_computed_vel_legsINcontacts_all.m";
    file3.open(file_name.c_str());
    file3<<"computed_vel = ["<<std::endl;

    std::ofstream file8;
    file_name = "testDynamics_computed_vel_l_leg.m";
    file8.open(file_name.c_str());
    file8<<"computed_dq_l_leg = ["<<std::endl;

    std::ofstream file9;
    file_name = "testDynamics_computed_vel_r_leg.m";
    file9.open(file_name.c_str());
    file9<<"computed_dq_r_leg = ["<<std::endl;

    std::ofstream file4;
    file_name = "testDynamics_torque_l_leg.m";
    file4.open(file_name.c_str());
    file4<<"tau_l_leg = ["<<std::endl;

    std::ofstream file5;
    file_name = "testDynamics_torque_r_leg.m";
    file5.open(file_name.c_str());
    file5<<"tau_r_leg = ["<<std::endl;

    std::ofstream file6;
    file_name = "testDynamics_filtered_ft_left_right.m";
    file6.open(file_name.c_str());
    file6<<"filtered_ft_left_right = ["<<std::endl;

    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
        yarp::sig::Vector tau = sensed_torque_exp[i];
        yarp::sig::Vector dq = computed_velocity_exp[i];
        file1<<tau.toString()<<std::endl;
        file2<<com_cartesian_error_exp[i].toString()<<std::endl;
        file3<<computed_velocity_exp[i].toString()<<std::endl;
        file4<<tau.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
                coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<std::endl;
        file5<<tau.subVector(coman_robot.idynutils.right_leg.joint_numbers[0],
                coman_robot.idynutils.right_leg.joint_numbers[5]).toString()<<std::endl;
        file6<<filtered_ft_left_right[i].toString()<<std::endl;
        file8<<dq.subVector(coman_robot.idynutils.left_leg.joint_numbers[0],
                coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<std::endl;
        file9<<dq.subVector(coman_robot.idynutils.right_leg.joint_numbers[0],
                coman_robot.idynutils.right_leg.joint_numbers[5]).toString()<<std::endl;
    }
    file1<<"];"<<std::endl;
    file1.close();
    file2<<"];"<<std::endl;
    file2.close();
    file3<<"];"<<std::endl;
    file3.close();
    file4<<"];"<<std::endl;
    file4.close();
    file5<<"];"<<std::endl;
    file5.close();
    file6<<"];"<<std::endl;
    file6.close();
    file8<<"];"<<std::endl;
    file8.close();
    file9<<"];"<<std::endl;
    file9.close();


    std::cout<<"PARAMETERS:"<<std::endl;
    std::cout<<"dT: "<<dT<<" [sec]"<<std::endl;
    std::cout<<"joint_vel_limits: "<<joint_vel_limits<<" [rad/sec]"<<std::endl;
    std::cout<<"dyn_constr_bound_scaling: "<<dyn_constr_bound_scaling<<std::endl;
    std::cout<<"dyn_constr_clip: "<<dyn_constr_clip<<std::endl;
    std::cout<<"torque_scaling_factor: "<<torque_scaling_factor<<std::endl;
    std::cout<<"eps regulation: "<<eps<<std::endl;
    std::cout<<"ft_filter: "<<ft_filter<<std::endl;
    std::cout<<std::endl;

    std::ofstream file7;
    file_name = "testConstraintWithContacts_externalForces.m";
    file7.open(file_name.c_str());
    file7<<"dT = "<<dT<<std::endl;
    file7<<"joint_vel_limits = "<<joint_vel_limits<<std::endl;
    file7<<"dyn_constr_bound_scaling = "<<dyn_constr_bound_scaling<<std::endl;
    file7<<"dyn_constr_clip = "<<dyn_constr_clip<<std::endl;
    file7<<"torque_scaling_factor = "<<torque_scaling_factor<<std::endl;
    file7<<"eps_regulation = "<<eps<<std::endl;
    file7<<"ft_filter = "<<ft_filter<<std::endl;
    file7.close();

    std::ofstream file10;
    file_name = "max_torques_legs.m";
    file10.open(file_name.c_str());
    file10<<"tau_legs_max = ["<<std::endl;
    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i)
        file10<<(Dyn->getTorqueLimits()).subVector(
            coman_robot.idynutils.left_leg.joint_numbers[0],
            coman_robot.idynutils.left_leg.joint_numbers[5]).toString()<<std::endl;
    file10<<"];"<<std::endl;
    file10.close();




    sleep(5);
    bool success = tests_utils::stopGazebo();
    if(success)
        std::cout<<"GAZEBO KILLED"<<std::endl;
    sleep(5);
    success = tests_utils::stopYarpServer();
    if(success)
        std::cout<<"yarpserver KILLED"<<std::endl;
    sleep(5);
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
