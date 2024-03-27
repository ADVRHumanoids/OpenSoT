#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/iHQP.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <xbot2_interface/collision.h>
#include <chrono>
#include <OpenSoT/utils/AutoStack.h>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include "collision_utils.h"
#define ENABLE_ROS true

#if ENABLE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#endif

#if ROS_VERSION_MINOR <= 12
#define STATIC_POINTER_CAST boost::static_pointer_cast
#define DYNAMIC_POINTER_CAST boost::dynamic_pointer_cast
#define SHARED_PTR boost::shared_ptr
#define MAKE_SHARED boost::make_shared
#else
#define STATIC_POINTER_CAST std::static_pointer_cast
#define DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
#define SHARED_PTR std::shared_ptr
#define MAKE_SHARED std::make_shared
#endif

namespace {

class testCollisionAvoidanceConstraint : public ::testing::Test
{

public:

    std::string ReadFile(std::string path)
    {
        std::ifstream t(path);
        std::stringstream buffer;
        buffer << t.rdbuf();
        return buffer.str();
    }


  testCollisionAvoidanceConstraint()
  {
      std::string urdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman_capsules.rviz";
      std::ifstream f(urdf_capsule_path);
      std::stringstream ss;
      ss << f.rdbuf();

      urdf = std::make_shared<urdf::Model>();
      urdf->initFile(urdf_capsule_path);

      std::string srdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman.srdf";
      srdf = std::make_shared<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);



      _model_ptr = XBot::ModelInterface::getModel(ReadFile(urdf_capsule_path), ReadFile(srdf_capsule_path), "pin");

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

      q = _model_ptr->getNeutralQ();
      _model_ptr->setJointPosition(q);
      _model_ptr->update();

#if ENABLE_ROS
      int argc = 0;
      char **argv;
      ros::init(argc, argv, "collision_avoidance_environment_test");
      n.reset(new ros::NodeHandle());
      pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000,1);

      KDL::Tree my_tree;
      if (!kdl_parser::treeFromFile(urdf_capsule_path, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");}
      rsp = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_tree);
      n->setParam("/robot_description", ss.str());

      for(unsigned int i = 0; i < this->_model_ptr->getJointNames().size(); ++i){
          joint_state.name.push_back(this->_model_ptr->getJointNames()[i]);
          joint_state.position.push_back(0.0);}
#endif

  }

  virtual ~testCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

#if ENABLE_ROS
public:
  void publishJointStates(const Eigen::VectorXd& q)
  {
      std::map<std::string, double> joint_map;
      for(unsigned int i = 0; i < q.size(); ++i){
          joint_state.position[i] = q[i];
          joint_map[joint_state.name[i]] = joint_state.position[i];
      }
      joint_state.header.stamp = ros::Time::now();

      pub.publish(joint_state);

      rsp->publishTransforms(joint_map, ros::Time::now(), "");
      rsp->publishFixedTransforms("");

      ros::spinOnce();
  }
#endif



  XBot::ModelInterface::Ptr _model_ptr;
  Eigen::VectorXd q;

  urdf::ModelSharedPtr urdf;
  srdf::ModelSharedPtr srdf;

#if ENABLE_ROS
  ///ROS
  std::shared_ptr<ros::NodeHandle> n;
  ros::Publisher pub;
  sensor_msgs::JointState joint_state;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp;
#endif

};

Eigen::VectorXd getGoodInitialPosition(const XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();
    _q[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LShYaw")] = -15.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getQIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RShYaw")] = 15.0*M_PI/180.0;
    _q[_model_ptr->getQIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}

TEST_F(testCollisionAvoidanceConstraint, testEnvironmentCollisionAvoidance){

    q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

#if ENABLE_ROS
        this->publishJointStates(q);
        this->publishJointStates(q);
        this->publishJointStates(q);



    sleep(1);
#endif

    string base_link = "torso";
    string left_arm_link = "LSoftHandLink";
    auto left_arm_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + left_arm_link,
                               *_model_ptr,
                               left_arm_link,
                               base_link
                             );
    Eigen::Affine3d left_arm_initial_pose;
    _model_ptr->getPose ( left_arm_link, base_link, left_arm_initial_pose );
    std::cout<<"left_arm_initial_pose: "<<left_arm_initial_pose.matrix()<<std::endl;

    string right_arm_link = "RSoftHandLink";
    auto right_arm_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + right_arm_link,
                               *_model_ptr,
                               right_arm_link,
                               base_link
                             );
    Eigen::Affine3d right_arm_initial_pose;
    _model_ptr->getPose ( right_arm_link, base_link, right_arm_initial_pose );
    std::cout<<"right_arm_initial_pose: "<<right_arm_initial_pose.matrix()<<std::endl;

    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits ( q_min, q_max );
    auto joint_limit_constraint = std::make_shared<OpenSoT::constraints::velocity::JointLimits> ( *_model_ptr, q_max, q_min );

    auto joint_velocity_limit_constraint = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits> ( *_model_ptr, 1., 0.005);


    OpenSoT::constraints::velocity::CollisionAvoidance::Ptr environment_collsion_constraint =
            std::make_shared<OpenSoT::constraints::velocity::CollisionAvoidance> (
                *_model_ptr, -1, this->urdf, this->srdf);


    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == environment_collsion_constraint->getCollisionJacobian().rows());
    unsigned int max_pairs = environment_collsion_constraint->getAineq().rows();

    // we consider only environment collision avoidance
    environment_collsion_constraint->setCollisionList(std::set<std::pair<std::string, std::string>>());
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == 0);

    Eigen::Affine3d w_T_c; w_T_c.setIdentity();
    w_T_c.translation()<< 0.7, 0, 0.;
    XBot::Collision::Shape::Box box;
    box.size<<0.1, 0.6, 1.4;

    EXPECT_TRUE(environment_collsion_constraint->addCollisionShape("mybox", "world", box, w_T_c));
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == 39)<<"Links are "<<39<<
                                                                                                 " WHILE environment_collsion_constraint->getCollisionJacobian().rows(): "<<
                                                                                                 environment_collsion_constraint->getCollisionJacobian().rows()<<std::endl;

    std::set<std::string> interested_links = {"LShp","LShr","LShy","LElb","LForearm","LSoftHandLink"};

    environment_collsion_constraint->setLinksVsEnvironment(interested_links);
    environment_collsion_constraint->update();
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == interested_links.size())<<"environment_collsion_constraint->getCollisionJacobian().rows(): "<<
                                                                                                            environment_collsion_constraint->getCollisionJacobian().rows()<<" WHILE "<<
                                                                                                           "interested_links.size(): "<<interested_links.size()<<std::endl;

    max_pairs = 300;
    environment_collsion_constraint->setMaxPairs(max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getAineq().rows() == max_pairs);
    EXPECT_TRUE(environment_collsion_constraint->getCollisionJacobian().rows() == interested_links.size());



    environment_collsion_constraint->setDetectionThreshold(1.);
    environment_collsion_constraint->setLinkPairThreshold(0.0001);
    environment_collsion_constraint->setBoundScaling(1.);





    auto autostack_ = std::make_shared<OpenSoT::AutoStack> ( left_arm_task + right_arm_task); // + 0.2*postural_task%indices
    autostack_ << joint_limit_constraint;
    autostack_ << environment_collsion_constraint;
    autostack_<<joint_velocity_limit_constraint;

    /* Create solver */
   double eps_regularization = 1e6;
   OpenSoT::solvers::solver_back_ends solver_backend = OpenSoT::solvers::solver_back_ends::qpOASES;
    auto solver = std::make_shared<OpenSoT::solvers::iHQP> ( autostack_->getStack(),
                     autostack_->getBounds(),
                     eps_regularization,
                     solver_backend );


#if ENABLE_ROS
    /* visualization */
        ros::Publisher marker_pub = n->advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
        visualization_msgs::Marker cube;
        cube.header.frame_id = "world";
        cube.header.stamp = ros::Time::now();
        cube.ns = "environment";
        cube.action = visualization_msgs::Marker::ADD;
        cube.id = 0;
        cube.type = visualization_msgs::Marker::CUBE;

        cube.scale.x = 0.1;
        cube.scale.y = 0.6;
        cube.scale.z = 1.4;

        cube.color.g = 1.0;
        cube.color.a = 0.5;

        tf::poseEigenToMsg ( w_T_c, cube.pose );
#endif


    double dt = 0.005; //[s]
    double T = 5; //[s]

    Eigen::VectorXd dq;
    dq.setZero(_model_ptr->getNv());
    for(unsigned int i = 0; i <= int(T/dt); ++i)
    {

        double t = i*dt;

        _model_ptr->setJointPosition ( q );
        _model_ptr->update();

        double length = 0.2;
        double period = 3;
        Eigen::Affine3d desired_pose;
        desired_pose.linear() = left_arm_initial_pose.linear();
        desired_pose.translation() = left_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,1 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
        left_arm_task->setReference ( desired_pose.matrix() );

        desired_pose.linear() = right_arm_initial_pose.linear();
        desired_pose.translation() = right_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,1 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
        right_arm_task->setReference ( desired_pose.matrix() );


        autostack_->update ();
        EXPECT_TRUE(solver->solve ( dq ));
        q = _model_ptr->sum(q, dq);

#if ENABLE_ROS
        this->publishJointStates(q);
        marker_pub.publish ( cube );
        usleep(30000);
#endif

    }

    /**
     * Due to environment the final y position of the left arm should be < than the final position of the right arm
     */
    Eigen::Affine3d w_T_torso;
    _model_ptr->getPose("torso", w_T_torso);
    Eigen::Affine3d w_T_la = w_T_torso*Eigen::Affine3d(left_arm_task->getActualPose());
    Eigen::Affine3d w_T_ra = w_T_torso*Eigen::Affine3d(right_arm_task->getActualPose());

    EXPECT_NEAR(std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0]), 0.0970416, 1e-7); //checked empirically...
    std::cout<<"std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0]): "<<std::fabs(w_T_la.translation()[0] - w_T_ra.translation()[0])<<std::endl;
    std::cout<<"w_T_la.translation()[0]: "<<w_T_la.translation()[0]<<std::endl;
    std::cout<<"w_T_ra.translation()[0]: "<<w_T_ra.translation()[0]<<std::endl;




}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
