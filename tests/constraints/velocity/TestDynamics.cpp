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

#define VISUALIZE_SIMULATION true

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
    OpenSoT::constraints::velocity::Dynamics constr(q,q,q,coman,3);
    std::vector<std::string> ft_in_contact;
    constr.crawlLinks(ft_links,
                      std::vector<std::string> { std::begin(links_in_contact), std::end(links_in_contact) },
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),2);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[0]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links[1]);

    for(unsigned int i = 0; i < 4; ++i)
        links_in_contact.pop_front();

    std::cout<<std::endl;
    std::cout<<"links in contact: "<<std::endl;
    for(link = links_in_contact.begin(); link != links_in_contact.end(); link++)
        std::cout<<"    "<<*link<<std::endl;

    constr.crawlLinks(ft_links,
                      std::vector<std::string> { std::begin(links_in_contact), std::end(links_in_contact) },
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

    constr.crawlLinks(ft_links,
                      std::vector<std::string> { std::begin(links_in_contact), std::end(links_in_contact) },
                      coman, ft_in_contact);
    std::cout<<"FT IN CONTACT: "<<std::endl;
    for(unsigned int i = 0; i < ft_in_contact.size(); ++i)
        std::cout<<"    "<<ft_in_contact[i]<<std::endl;

    EXPECT_EQ(ft_in_contact.size(),2);
    EXPECT_TRUE(ft_in_contact[0] == ft_links[1]);
    EXPECT_TRUE(ft_in_contact[1] == ft_links[3]);
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

TEST_F(testDynamicsConstr, testConstraint) {

    // Start YARP Server
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman_fixed.world";
    if(VISUALIZE_SIMULATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);

    sleep(4);



for(unsigned int j = 0; j < 2; ++j){
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

    double dT = 0.001;
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(0.6, dT,q.size()));


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

    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    constraints::velocity::Dynamics::Ptr Dyn;
    if(j == 1)
    {
        yarp::sig::Vector zero(q.size(), 0.0);
        Dyn = constraints::velocity::Dynamics::Ptr(
                new constraints::velocity::Dynamics(q,zero,
                    0.9*coman_robot.idynutils.iDyn3_model.getJointTorqueMax(),
                    coman_robot.idynutils, dT));
    }

    double eps = 1e10;

    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
    if(j == 0)
        sot = solvers::QPOases_sot::Ptr(
                new solvers::QPOases_sot(stack_of_tasks, bounds, eps));
    else
        sot = solvers::QPOases_sot::Ptr(
                   new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));


    yarp::sig::Vector dq(q.size(), 0.0);
    std::vector<yarp::sig::Vector> sensed_torque_exp;
    std::vector<yarp::sig::Vector> cartesian_error_exp;
    std::vector<yarp::sig::Vector> computed_velocity_exp;
    int steps = 10000;
    sensed_torque_exp.reserve(steps);
    cartesian_error_exp.reserve(steps);
    computed_velocity_exp.reserve(steps);
    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, true);

        bounds->update(q);
        taskCartesianAggregated->update(q);
        postural_task->update(q);

        cartesian_error_exp.push_back(cartesian_task_l_wrist->getError());

        if(j == 1)
            Dyn->update(cat(q,dq/dT));

        if(sot->solve(dq)){
            computed_velocity_exp.push_back(dq);
            q += dq;}
        coman_robot.move(q);
        double toc = yarp::os::Time::now();

        yarp::sig::Vector q_sensed(q.size(), 0.0);
        yarp::sig::Vector dq_sensed(q.size(), 0.0);
        yarp::sig::Vector tau_sensed(q.size(), 0.0);
        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);
        sensed_torque_exp.push_back(tau_sensed);

        if((toc-tic) < dT)
            usleep(dT*1000.0 - (toc-tic)*1E-6);
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

}




    tests_utils::stopGazebo();
    sleep(5);
    tests_utils::stopYarpServer();
}




TEST_F(testDynamicsConstr, testConstraintWithContacts) {

    // Start YARP Server
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    if(VISUALIZE_SIMULATION)
        tests_utils::startGazebo(world_path);
    else
        tests_utils::startGZServer(world_path);

    sleep(4);



for(unsigned int j = 0; j < 2; ++j){
    sleep(5);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");


    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

    //Homing
    coman_robot.idynutils.updateiDyn3Model(q,true);
    coman_robot.setPositionMode();
    yarp::sig::Vector legs_speed(6,0.3);
    legs_speed[3] = 2.0*legs_speed[3];
    coman_robot.left_leg.setReferenceSpeeds(legs_speed);
    coman_robot.right_leg.setReferenceSpeeds(legs_speed);
    coman_robot.left_arm.setReferenceSpeed(0.3);
    coman_robot.right_arm.setReferenceSpeed(0.3);
    coman_robot.torso.setReferenceSpeed(0.3);
    coman_robot.move(q);

//    //Set Up SoT
//    sleep(5);
//    // BOUNDS
//    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
//            constraints::velocity::JointLimits::ConstraintPtr(
//                new constraints::velocity::JointLimits(
//                    q,
//                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
//                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

//    double dT = 0.001;
//    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
//            constraints::velocity::VelocityLimits::ConstraintPtr(
//                new constraints::velocity::VelocityLimits(0.6, dT,q.size()));


//    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
//                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
//                                            q.size()));
//    // TASKS
//    tasks::velocity::Cartesian::Ptr cartesian_task_l_wrist=
//            tasks::velocity::Cartesian::Ptr(
//                new tasks::velocity::Cartesian("cartesian::l_wrist", q,
//                    coman_robot.idynutils,"l_wrist", "Waist"));
//    Matrix goal = cartesian_task_l_wrist->getActualPose();
//    goal(0,3) += 0.5;
//    cartesian_task_l_wrist->setReference(goal);

//    tasks::velocity::Cartesian::Ptr cartesian_task_r_wrist=
//            tasks::velocity::Cartesian::Ptr(
//                new tasks::velocity::Cartesian("cartesian::r_wrist", q,
//                    coman_robot.idynutils,"r_wrist", "world"));

//    std::list<tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
//    cartesianTasks.push_back(cartesian_task_l_wrist);
//    cartesianTasks.push_back(cartesian_task_r_wrist);
//    Task<Matrix, Vector>::TaskPtr taskCartesianAggregated =
//            tasks::Aggregated::TaskPtr(
//       new tasks::Aggregated(cartesianTasks,q.size()));

//    tasks::velocity::Postural::Ptr postural_task=
//            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

//    solvers::QPOases_sot::Stack stack_of_tasks;
//    stack_of_tasks.push_back(taskCartesianAggregated);
//    stack_of_tasks.push_back(postural_task);

//    constraints::velocity::Dynamics::Ptr Dyn;
//    if(j == 1)
//    {
//        yarp::sig::Vector zero(q.size(), 0.0);
//        Dyn = constraints::velocity::Dynamics::Ptr(
//                new constraints::velocity::Dynamics(q,zero,
//                    0.9*coman_robot.idynutils.iDyn3_model.getJointTorqueMax(),
//                    coman_robot.idynutils, dT));
//    }

//    double eps = 1e10;

//    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
//    if(j == 0)
//        sot = solvers::QPOases_sot::Ptr(
//                new solvers::QPOases_sot(stack_of_tasks, bounds, eps));
//    else
//        sot = solvers::QPOases_sot::Ptr(
//                   new solvers::QPOases_sot(stack_of_tasks, bounds, Dyn, eps));


//    yarp::sig::Vector dq(q.size(), 0.0);
//    std::vector<yarp::sig::Vector> sensed_torque_exp;
//    std::vector<yarp::sig::Vector> cartesian_error_exp;
//    std::vector<yarp::sig::Vector> computed_velocity_exp;
//    int steps = 10000;
//    sensed_torque_exp.reserve(steps);
//    cartesian_error_exp.reserve(steps);
//    computed_velocity_exp.reserve(steps);
//    for(unsigned int i = 0; i < steps; ++i)
//    {
//        double tic = yarp::os::Time::now();
//        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, true);

//        bounds->update(q);
//        taskCartesianAggregated->update(q);
//        postural_task->update(q);

//        cartesian_error_exp.push_back(cartesian_task_l_wrist->getError());

//        if(j == 1)
//            Dyn->update(cat(q,dq/dT));

//        if(sot->solve(dq)){
//            computed_velocity_exp.push_back(dq);
//            q += dq;}
//        coman_robot.move(q);
//        double toc = yarp::os::Time::now();

//        yarp::sig::Vector q_sensed(q.size(), 0.0);
//        yarp::sig::Vector dq_sensed(q.size(), 0.0);
//        yarp::sig::Vector tau_sensed(q.size(), 0.0);
//        coman_robot.sense(q_sensed, dq_sensed, tau_sensed);
//        sensed_torque_exp.push_back(tau_sensed);

//        if((toc-tic) < dT)
//            usleep(dT*1000.0 - (toc-tic)*1E-6);
//    }

//    std::ofstream file1;
//    std::string file_name = "testDynamics_torque_exp_"+std::to_string(j)+"_left_arm.m";
//    file1.open(file_name);
//    file1<<"tau_"<<j<<" = ["<<std::endl;

//    std::ofstream file2;
//    file_name = "testDynamics_cartesian_error_exp_"+std::to_string(j)+"_left_arm.m";
//    file2.open(file_name);
//    file2<<"cartesian_error_"<<j<<" = ["<<std::endl;

//    std::ofstream file3;
//    file_name = "testDynamics_computed_vel_exp_"+std::to_string(j)+"_left_arm.m";
//    file3.open(file_name);
//    file3<<"computed_vel_"<<j<<" = ["<<std::endl;

//    for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
//        yarp::sig::Vector tau = sensed_torque_exp[i];
//        file1<<yarp::math::cat(
//                   tau.subVector(coman_robot.idynutils.torso.joint_numbers[0],
//                                 coman_robot.idynutils.torso.joint_numbers[2]),
//                tau.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
//                              coman_robot.idynutils.left_arm.joint_numbers[6])
//                ).toString()<<std::endl;
//        file2<<cartesian_error_exp[i].toString()<<std::endl;
//        file3<<yarp::math::cat(
//                computed_velocity_exp[i].subVector(coman_robot.idynutils.torso.joint_numbers[0],
//                                 coman_robot.idynutils.torso.joint_numbers[2]),
//                computed_velocity_exp[i].subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
//                              coman_robot.idynutils.left_arm.joint_numbers[6])
//                ).toString()<<std::endl;
//    }
//    file1<<"];"<<std::endl;
//    file1.close();
//    file2<<"];"<<std::endl;
//    file2.close();
//    file3<<"];"<<std::endl;
//    file3.close();

//    if(j == 1){
//        for(unsigned int i = 0; i < sensed_torque_exp.size(); ++i){
//            yarp::sig::Vector t = sensed_torque_exp[i];
//            yarp::sig::Vector x = yarp::math::cat(
//                               t.subVector(coman_robot.idynutils.torso.joint_numbers[0],
//                                             coman_robot.idynutils.torso.joint_numbers[2]),
//                               t.subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
//                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
//            yarp::sig::Vector xx = yarp::math::cat(
//                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.torso.joint_numbers[0],
//                                             coman_robot.idynutils.torso.joint_numbers[2]),
//                               coman_robot.idynutils.iDyn3_model.getJointTorqueMax().subVector(coman_robot.idynutils.left_arm.joint_numbers[0],
//                                          coman_robot.idynutils.left_arm.joint_numbers[6]));
//            for(unsigned int jj = 0; jj < x.size(); ++jj)
//                EXPECT_LE(fabs(x[jj]), 0.9*xx[jj])<<"@joint "<<jj;
        }
    }




}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
