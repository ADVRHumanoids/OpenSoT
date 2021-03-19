#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/iHQP.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <OpenSoT/utils/collision_utils.h>
#include <chrono>
#include <OpenSoT/utils/AutoStack.h>
#include <eigen_conversions/eigen_msg.h>
#define ENABLE_ROS true

#if ENABLE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#endif

namespace {

class testCollisionAvoidanceConstraint : public ::testing::Test{
public:

 protected:

  testCollisionAvoidanceConstraint()
  {
      std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
      std::string relative_path = "/external/OpenSoT/tests/configs/bigman/configs/config_bigman.yaml";
      std::string urdf_path = robotology_root + "/external/OpenSoT/tests/robots/bigman/bigman_capsules.rviz";
      std::ifstream f(urdf_path);
      std::stringstream ss;
      ss << f.rdbuf();

      urdf = boost::make_shared<urdf::Model>();
      urdf->initFile(urdf_path);

      std::string srdf_capsule_path = robotology_root + "/external/OpenSoT/tests/robots/bigman/bigman.srdf";
      srdf = boost::make_shared<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);


      _path_to_cfg = robotology_root + relative_path;

      _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

      q.resize(_model_ptr->getJointNum());
      q.setZero(q.size());

#if ENABLE_ROS
      int argc = 0;
      char **argv;
      ros::init(argc, argv, "collision_avoidance_environment_test");
      n.reset(new ros::NodeHandle());
      pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000,1);

      KDL::Tree my_tree;
      if (!kdl_parser::treeFromFile(urdf_path, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");}
      rsp = boost::make_shared<robot_state_publisher::RobotStatePublisher>(my_tree);
      n->setParam("/robot_description", ss.str());

      for(unsigned int i = 0; i < this->_model_ptr->getEnabledJointNames().size(); ++i){
          joint_state.name.push_back(this->_model_ptr->getEnabledJointNames()[i]);
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
  std::string _path_to_cfg;
  Eigen::VectorXd q;

  urdf::ModelSharedPtr urdf;
  srdf::ModelSharedPtr srdf;

#if ENABLE_ROS
  ///ROS
  boost::shared_ptr<ros::NodeHandle> n;
  ros::Publisher pub;
  sensor_msgs::JointState joint_state;
  boost::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp;
#endif

};

Eigen::VectorXd getGoodInitialPosition(const XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q(_model_ptr->getJointNum());
    _q.setZero(_q.size());
    _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

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
    auto left_arm_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + left_arm_link,
                               q,
                               *_model_ptr,
                               left_arm_link,
                               base_link
                             );
    Eigen::Affine3d left_arm_initial_pose;
    _model_ptr->getPose ( left_arm_link, base_link, left_arm_initial_pose );
    std::cout<<"left_arm_initial_pose: "<<left_arm_initial_pose.matrix()<<std::endl;

    string right_arm_link = "RSoftHandLink";
    auto right_arm_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
                             ( base_link + "_TO_" + right_arm_link,
                               q,
                               *_model_ptr,
                               right_arm_link,
                               base_link
                             );
    Eigen::Affine3d right_arm_initial_pose;
    _model_ptr->getPose ( right_arm_link, base_link, right_arm_initial_pose );
    std::cout<<"right_arm_initial_pose: "<<right_arm_initial_pose.matrix()<<std::endl;

    Eigen::VectorXd q_min, q_max;
    _model_ptr->getJointLimits ( q_min, q_max );
    auto joint_limit_constraint = boost::make_shared<OpenSoT::constraints::velocity::JointLimits> ( q, q_max, q_min );


    std::vector<std::string> interested_links = {"LShp","LShr","LShy","LElb","LForearm","LSoftHandLink"};
    std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd>> envionment_collision_objects;
    std::shared_ptr<fcl::CollisionGeometryd> shape = std::make_shared<fcl::Boxd> ( 0.1, 0.6, 1.4 );
    boost::shared_ptr<fcl::CollisionObjectd> collision_object ( new fcl::CollisionObjectd ( shape ) );
    fcl::Transform3d shape_origin;
    shape_origin.translation() << 0.75, 0, 0.; // in world frame
    shape_origin.linear() = Eigen::Matrix3d::Identity();
    collision_object->setTransform ( shape_origin );
    envionment_collision_objects["env"] = collision_object;

    OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr environment_collsion_constraint =
            boost::make_shared<OpenSoT::constraints::velocity::SelfCollisionAvoidance> (
                q, *_model_ptr, -1, this->urdf, this->srdf);

    environment_collsion_constraint->setDetectionThreshold(1.);
    environment_collsion_constraint->setLinkPairThreshold(0.1);
    environment_collsion_constraint->setBoundScaling(1.);




    for(auto pair : envionment_collision_objects)
        EXPECT_TRUE(environment_collsion_constraint->addWorldCollision(pair.first, pair.second));


    // we consider only environment collision avoidance
    environment_collsion_constraint->setCollisionWhiteList(std::list<LinkPairDistance::LinksPair>());
    environment_collsion_constraint->setLinksVsEnvironment(interested_links);


    auto autostack_ = boost::make_shared<OpenSoT::AutoStack> ( left_arm_task + right_arm_task); // + 0.2*postural_task%indices
    autostack_ << joint_limit_constraint;
    autostack_ << environment_collsion_constraint;

    /* Create solver */
   double eps_regularization = 1e6;
   OpenSoT::solvers::solver_back_ends solver_backend = OpenSoT::solvers::solver_back_ends::qpOASES;
    auto solver = boost::make_shared<OpenSoT::solvers::iHQP> ( autostack_->getStack(),
                     autostack_->getBounds(),
                     eps_regularization,
                     solver_backend );

    Eigen::Affine3d collision_pose;
    collision_pose.translation() = shape_origin.translation(); // in world frame
    collision_pose.linear() = shape_origin.linear();

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

        tf::poseEigenToMsg ( collision_pose, cube.pose );
        cube.pose.position.x -= 0.05;
#endif


    double dt = 0.005; //[s]
    double T = 5; //[s]

    Eigen::VectorXd dq;
    dq.setZero(q.size());
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


        EXPECT_LE(left_arm_task->getActualPose()(0,3), 0.243);

        autostack_->update ( q );
        EXPECT_TRUE(solver->solve ( dq ));
        q += dq;

#if ENABLE_ROS
        this->publishJointStates(q);
        marker_pub.publish ( cube );
        usleep(30000);
#endif

    }



}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
