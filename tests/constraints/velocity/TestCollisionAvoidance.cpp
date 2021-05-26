#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/iHQP.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <OpenSoT/utils/collision_utils.h>
#include <chrono>
#define ENABLE_ROS false

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

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <eigen_conversions/eigen_kdl.h>
// tf::vectorKDLToEigen
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i];
}

#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)
#define SMALL_NUM 1e-5

KDL::Frame fcl2KDL(const fcl::Transform3<double> &in)
{
    Eigen::Quaterniond q(in.linear());
    Eigen::Vector3d t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    return f;
}

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


double dist3D_Segment_to_Segment (const Eigen::Vector3d & segment_A_endpoint_1,
                                  const Eigen::Vector3d & segment_A_endpoint_2,
                                  const Eigen::Vector3d & segment_B_endpoint_1,
                                  const Eigen::Vector3d & segment_B_endpoint_2,
                                  Eigen::Vector3d & closest_point_on_segment_A,
                                  Eigen::Vector3d & closest_point_on_segment_B)
{

    using namespace Eigen;

    Vector3d   u = segment_A_endpoint_2 - segment_A_endpoint_1;
    Vector3d   v = segment_B_endpoint_2 - segment_B_endpoint_1;
    Vector3d   w = segment_A_endpoint_1 - segment_B_endpoint_1;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (std::fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (std::fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    closest_point_on_segment_A = segment_A_endpoint_1 + sc * u;
    closest_point_on_segment_B = segment_B_endpoint_1 + tc * v;

//    std::cout << "CP1: " << std::endl << CP1 << std::endl;
//    std::cout << "CP2: " << std::endl << CP2 << std::endl;

    // get the difference of the two closest points
    Vector3d   dP = closest_point_on_segment_A - closest_point_on_segment_B;  // =  S1(sc) - S2(tc)

    double Dm = dP.norm();   // return the closest distance

    // I leave the line here for observing the minimum distance between the inner line segments of the corresponding capsule pair
    //std::cout << "Dm: " << std::endl << Dm << std::endl;

    return Dm;
}

class TestCapsuleLinksDistance
{

private:
    ComputeLinksDistance& _computeDistance;

public:

    TestCapsuleLinksDistance(ComputeLinksDistance& computeDistance)
        :_computeDistance(computeDistance)
    {

    }


    bool globalToLinkCoordinates(const std::string& linkName,
                                 const fcl::Transform3<double> &fcl_w_T_f,
                                 KDL::Frame &link_T_f)
    {

        return _computeDistance.globalToLinkCoordinates(linkName, fcl_w_T_f, link_T_f);
    }

    bool globalToLinkCoordinatesKDL(const std::string& linkName,
                                    const fcl::Transform3<double> &fcl_w_T_f,
                                    KDL::Frame &link_T_f)
    {

        KDL::Frame w_T_f = fcl2KDL(fcl_w_T_f);

        fcl::Transform3<double> fcl_w_T_shape = _computeDistance.getCollisionObjects()[linkName]->getTransform();
        KDL::Frame w_T_shape = fcl2KDL(fcl_w_T_shape);

        KDL::Frame shape_T_f = w_T_shape.Inverse()*w_T_f;

        link_T_f = _computeDistance.getLinkToShapeTransforms()[linkName] * shape_T_f;

        return true;
    }

};

namespace{


class testSelfCollisionAvoidanceConstraint : public ::testing::Test{
public:

#if ENABLE_ROS
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


 protected:

  testSelfCollisionAvoidanceConstraint()
  {
#if ENABLE_ROS
      int argc = 0;
      char **argv;
      ros::init(argc, argv, "collision_avoidance_test");
      n.reset(new ros::NodeHandle());
      pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000);
      pub2 = n->advertise<visualization_msgs::Marker>("link_distances", 1, true);
#endif

      std::string relative_path = OPENSOT_TEST_PATH "configs/bigman/configs/config_bigman_capsules.yaml";
      std::string urdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman_capsules.rviz";
      std::ifstream f(urdf_capsule_path);
      std::stringstream ss;
      ss << f.rdbuf();

      urdf = MAKE_SHARED<urdf::Model>();
      urdf->initFile(urdf_capsule_path);

      std::string srdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman.srdf";
      srdf = MAKE_SHARED<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);


      _path_to_cfg = relative_path;

      _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

#if ENABLE_ROS
      KDL::Tree my_tree;
      if (!kdl_parser::treeFromFile(urdf_capsule_path, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");}
      rsp.reset(new robot_state_publisher::RobotStatePublisher(my_tree));
      n->setParam("/robot_description", ss.str());
#endif

      q.resize(_model_ptr->getJointNum());
      q.setZero(q.size());

      urdf = MAKE_SHARED<urdf::Model>();
      urdf->initFile(urdf_capsule_path);

      srdf = MAKE_SHARED<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);


      compute_distance = std::make_shared<ComputeLinksDistance>(*_model_ptr, urdf, srdf);

      sc_constraint = std::make_shared<OpenSoT::constraints::velocity::CollisionAvoidance>
              (q,
               *_model_ptr,
               -1,
               urdf,
               srdf);
      sc_constraint->setLinkPairThreshold(0.005);

#if ENABLE_ROS
      for(unsigned int i = 0; i < this->_model_ptr->getEnabledJointNames().size(); ++i){
          joint_state.name.push_back(this->_model_ptr->getEnabledJointNames()[i]);
          joint_state.position.push_back(0.0);}
#endif
  }

  virtual ~testSelfCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }



  XBot::ModelInterface::Ptr _model_ptr;
  std::string _path_to_cfg;
  Eigen::VectorXd q;
  std::shared_ptr<ComputeLinksDistance> compute_distance;
  OpenSoT::constraints::velocity::CollisionAvoidance::Ptr sc_constraint;
  urdf::ModelSharedPtr urdf;
  srdf::ModelSharedPtr srdf;

#if ENABLE_ROS
  ///ROS
  std::shared_ptr<ros::NodeHandle> n;
  ros::Publisher pub;
  ros::Publisher pub2;
  sensor_msgs::JointState joint_state;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp;
#endif

};


TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithoutSC){

    this->q = getGoodInitialPosition(this->_model_ptr);
    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_hand", this->q,
                                                        *(_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_hand", this->q,
                                                        *(_model_ptr.get()), linkB, "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    Eigen::MatrixXd T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    Eigen::MatrixXd T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    Eigen::MatrixXd T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    Eigen::MatrixXd T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
       new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(this->q));


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 10;
    Eigen::VectorXd qmin, qmax;
    this->_model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(this->q, qmax, qmin));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::iHQP::Ptr(
        new OpenSoT::solvers::iHQP(stack_of_tasks, bounds));

    Eigen::VectorXd dq(this->q.size()); dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq.setZero(dq.size());}
        this->q += dq;
    }

    std::cout << "Q_final: " << this->q.transpose() << std::endl;

    std::cout<<"Initial Left Arm: "<<T_init_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<T_reference_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<task_left_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<T_init_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<T_reference_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<task_right_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_left_arm->getActualPose()(i,j), T_reference_l_arm(i,j), 1E-4);

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_right_arm->getActualPose()(i,j), T_reference_r_arm(i,j), 1E-4);

    // check the distance betweem hands
    KDL::Frame w_T_link_left_hand, w_T_link_right_hand;
    _model_ptr->getPose(linkA, w_T_link_left_hand);
    _model_ptr->getPose(linkB, w_T_link_right_hand);


    double actual_distance = ( w_T_link_left_hand.p - w_T_link_right_hand.p ).Norm();

    EXPECT_NEAR(0.0, actual_distance, 1E-4);

}


TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithSC){

    this->q = getGoodInitialPosition(this->_model_ptr);
    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_hand", this->q,
                                                        *(_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_hand", this->q,
                                                        *(_model_ptr.get()), linkB, "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    Eigen::MatrixXd T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    Eigen::MatrixXd T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    Eigen::MatrixXd T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    Eigen::MatrixXd T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    std::cout << "xxx Setting whitelist" << std::endl;
    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
    this->sc_constraint->setCollisionWhiteList(whiteList);
    std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
                new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
    taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;

    Eigen::VectorXd qmin, qmax;
    this->_model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(this->q, qmax, qmin));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::iHQP::Ptr(
                new OpenSoT::solvers::iHQP(stack_of_tasks, bounds));

    Eigen::VectorXd dq(this->q.size()); dq.setZero(dq.size());
    for(unsigned int i = 0; i < 10*t; ++i)
    {
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        auto tic = std::chrono::steady_clock::now();
        this->sc_constraint->update(this->q);
        auto toc = std::chrono::steady_clock::now();
        auto time_for_update = std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count();
        std::cout<<"SCA Update time: "<<time_for_update/1000.<<" [ms]"<<std::endl;

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq.setZero(dq.size());}
        this->q += dq;

#if ENABLE_ROS
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time().now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;
    marker.color.a = 1.;
    marker.scale.x = 0.005;
    marker.scale.y = 0.;
    marker.scale.z = 0.;
    for(const auto& data : this->sc_constraint->getLinkPairDistances())
    {
        auto k2p = [](const KDL::Vector &k)->geometry_msgs::Point{
            geometry_msgs::Point p;
            p.x = k[0]; p.y = k[1]; p.z = k[2];
            return p;
        };



        // closest point on first link
        marker.points.push_back(k2p(data.getClosestPoints().first.p));
        // closest point on second link
        marker.points.push_back(k2p(data.getClosestPoints().second.p));
    }


        pub2.publish(marker);

        this->publishJointStates(this->q);
        usleep(100000);
#endif



    }

    std::cout << "Q_final: " << this->q.transpose() << std::endl;

    std::cout<<"Initial Left Arm: "<<T_init_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<T_reference_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<task_left_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<T_init_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<T_reference_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<task_right_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;

    // check the actual distance between the hand capsule pair
    std::shared_ptr<ComputeLinksDistance> compute_distance =
            std::make_shared<ComputeLinksDistance>(*_model_ptr, this->urdf, this->srdf);
    compute_distance->setCollisionWhiteList(whiteList);
    std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
    LinkPairDistance result = results.front();
    double reference_distance;
    reference_distance = result.getDistance();
    std::cout<<"reference_distance: "<<reference_distance<<std::endl;


    EXPECT_NEAR(0.005, reference_distance, 1e-3);


}


TEST_F(testSelfCollisionAvoidanceConstraint, testMultipleCapsulePairsSC){

    this->q = getGoodInitialPosition(this->_model_ptr);
    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();

#if ENABLE_ROS
    this->publishJointStates(this->q);
    this->publishJointStates(this->q);
    this->publishJointStates(this->q);


    sleep(1);
#endif

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    std::string linkC = "LFootmot";
    std::string linkD = "RFootmot";

    // arm task
    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, *(_model_ptr.get()),linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, *(_model_ptr.get()),linkB, "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    Eigen::MatrixXd T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    Eigen::MatrixXd T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    Eigen::MatrixXd T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    Eigen::MatrixXd T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    // leg task
    // Note: for now, we don't have the capsule information for the links LFoot and RFoot. In the meantime, we can only use these
    // links to realize the full posture kinematical contronl with enough DOFs (at least 6). So, we implement the Cartesian task for
    // these two links, while implement the self-collision avoidance constraint for the links LFootmot and RFootmot whose origin are
    // coincident with those of their child links, i.e., LFoot and RFoot.

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg", this->q, *(_model_ptr.get()),"LFoot", "Waist"));
    task_left_leg->setOrientationErrorGain(0.1);

    Eigen::MatrixXd W = task_left_leg->getWeight();
    //W(1,1) = 1.1;
    W(0,0) = 3.;
    W(2,2) = 3.;
    task_left_leg->setWeight(W);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg", this->q, *(_model_ptr.get()),"RFoot", "Waist"));
    task_right_leg->setOrientationErrorGain(0.1);

    task_right_leg->setWeight(W);

    Eigen::MatrixXd T_init_l_leg(4,4);
    T_init_l_leg = task_left_leg->getReference();

    Eigen::MatrixXd T_init_r_leg(4,4);
    T_init_r_leg = task_right_leg->getReference();

    Eigen::MatrixXd T_reference_l_leg(4,4);
    T_reference_l_leg = task_left_leg->getReference();
    T_reference_l_leg(1,3) = 0.0;
    task_left_leg->setReference(T_reference_l_leg);

    Eigen::MatrixXd T_reference_r_leg(4,4);
    T_reference_r_leg = task_right_leg->getReference();
    T_reference_r_leg(1,3) = 0.0;
    task_right_leg->setReference(T_reference_r_leg);

    // set whitelist

    std::cout << "xxx Setting whitelist" << std::endl;
    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
    whiteList.push_back(std::pair<std::string,std::string>(linkC,linkD));
    this->sc_constraint->setCollisionWhiteList(whiteList);
    std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    cartesianTasks.push_back(task_left_leg);
    cartesianTasks.push_back(task_right_leg);
    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
                new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
    taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 5;
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(this->q, qmax, qmin));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::iHQP::Ptr(
                new OpenSoT::solvers::iHQP(stack_of_tasks, bounds));

    Eigen::VectorXd dq(this->q.size()); dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq.setZero(dq.size());}
        this->q += dq;
#if ENABLE_ROS
        this->publishJointStates(this->q);
        usleep(50000);
#endif
    }

    std::cout << "Q_final: " << this->q.transpose() << std::endl;

    std::cout<<"Initial Left Arm: "<<T_init_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<T_reference_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<task_left_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<T_init_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<T_reference_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<task_right_arm->getActualPose()<<std::endl;

    // showing the data of the legs

    std::cout<<std::endl;

    std::cout<<"Initial Left leg: "<<T_init_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left leg: "<<T_reference_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left leg: "<<task_left_leg->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right leg: "<<T_init_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right leg: "<<T_reference_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right leg: "<<task_right_leg->getActualPose()<<std::endl;

    std::cout<<std::endl;

    // start checking the distances of the capsules

    typedef std::pair<std::string,std::string> CapsulePair;
    std::vector<CapsulePair> CasulePairs_vec;
    CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkA,linkB));
    CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkC,linkD));


    TestCapsuleLinksDistance compute_distance_observer(*(compute_distance.get()));

    int i;
    for (i=0; i < CasulePairs_vec.size(); i++)
    {

        std::string _linkA = CasulePairs_vec[i].first;
        std::string _linkB = CasulePairs_vec[i].second;

        //Note: the names of the variables below are not their literal meanings, just for convenience of the code writing

        std::shared_ptr<ComputeLinksDistance> compute_distance =
                std::make_shared<ComputeLinksDistance>(*_model_ptr, this->urdf, this->srdf);
        std::list<std::pair<std::string,std::string> > whiteList_;
        whiteList_.push_back(std::pair<std::string,std::string>(_linkA,_linkB));
        compute_distance->setCollisionWhiteList(whiteList_);
        std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
        LinkPairDistance result = results.front();
        double reference_distance;
        reference_distance = result.getDistance();

        if (i == 0)
        {
            std::cout << "checking the distance between hands" << std::endl;
            EXPECT_NEAR(0.005, reference_distance, 1e-4);
        }
        else if (i ==1)
        {
            std::cout << "checking the distance between legs" << std::endl;
            EXPECT_NEAR(0.005, reference_distance, 1e-4);
        }
        else
            std::cout << "The dimension of CasulePairs_vec is incorrect!" << std::endl;

        std::cout<<"reference_distance: "<<reference_distance<<std::endl;
    }


}


TEST_F(testSelfCollisionAvoidanceConstraint, testChangeWhitelistOnline){

    this->q = getGoodInitialPosition(this->_model_ptr);
    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    std::string linkC = "LFootmot";
    std::string linkD = "RFootmot";

    // arm task
    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, *(this->_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, *(this->_model_ptr.get()), linkB, "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    Eigen::MatrixXd T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    Eigen::MatrixXd T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    Eigen::MatrixXd T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    Eigen::MatrixXd T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    // leg task
    // Note: for now, we don't have the capsule information for the links LFoot and RFoot. In the meantime, we can only use these
    // links to realize the full posture kinematical contronl with enough DOFs (at least 6). So, we implement the Cartesian task for
    // these two links, while implement the self-collision avoidance constraint for the links LFootmot and RFootmot whose origin are
    // coincident with those of their child links, i.e., LFoot and RFoot.

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg", this->q, *(this->_model_ptr.get()),"LFoot", "Waist"));
    task_left_leg->setOrientationErrorGain(0.1);
    Eigen::MatrixXd W = task_left_leg->getWeight();
    //W(1,1) = 1.1;
    W(0,0) = 3.;
    W(2,2) = 3.;
    task_left_leg->setWeight(W);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg", this->q, *(this->_model_ptr.get()),"RFoot", "Waist"));
    task_right_leg->setOrientationErrorGain(0.1);
    task_right_leg->setWeight(W);

    Eigen::MatrixXd T_init_l_leg(4,4);
    T_init_l_leg = task_left_leg->getReference();

    Eigen::MatrixXd T_init_r_leg(4,4);
    T_init_r_leg = task_right_leg->getReference();

    Eigen::MatrixXd T_reference_l_leg(4,4);
    T_reference_l_leg = task_left_leg->getReference();
    T_reference_l_leg(1,3) = 0.0;
    task_left_leg->setReference(T_reference_l_leg);

    Eigen::MatrixXd T_reference_r_leg(4,4);
    T_reference_r_leg = task_right_leg->getReference();
    T_reference_r_leg(1,3) = 0.0;
    task_right_leg->setReference(T_reference_r_leg);

    // set whitelist

    std::cout << "xxx Setting whitelist" << std::endl;
    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
    whiteList.push_back(std::pair<std::string,std::string>(linkC,linkD));
    this->sc_constraint->setCollisionWhiteList(whiteList);
    std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    cartesianTasks.push_back(task_left_leg);
    cartesianTasks.push_back(task_right_leg);
    OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
                new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
    taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::iHQP::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 10;
    Eigen::VectorXd qmin,qmax;
    this->_model_ptr->getJointLimits(qmin,qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(q,qmax,qmin));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::iHQP::Ptr(
                new OpenSoT::solvers::iHQP(stack_of_tasks, bounds));

    Eigen::VectorXd dq(this->q.size()); dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq.setZero(dq.size());}
        this->q += dq;
#if ENABLE_ROS
        this->publishJointStates(this->q);
        usleep(10000);
#endif
    }

    std::cout << "Q_final: " << this->q.transpose() << std::endl;

    std::cout<<"Initial Left Arm: "<<T_init_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<T_reference_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<task_left_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<T_init_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<T_reference_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<task_right_arm->getActualPose()<<std::endl;

    // showing the data of the legs

    std::cout<<std::endl;

    std::cout<<"Initial Left leg: "<<T_init_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left leg: "<<T_reference_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left leg: "<<task_left_leg->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right leg: "<<T_init_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right leg: "<<T_reference_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right leg: "<<task_right_leg->getActualPose()<<std::endl;

    std::cout<<std::endl;

    // start checking the distances of the capsules

    typedef std::pair<std::string,std::string> CapsulePair;
    std::vector<CapsulePair> CasulePairs_vec;
    CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkA,linkB));
    CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkC,linkD));


    TestCapsuleLinksDistance compute_distance_observer(*(compute_distance.get()));

    for (int i=0; i < CasulePairs_vec.size(); i++)
    {

        std::string _linkA = CasulePairs_vec[i].first;
        std::string _linkB = CasulePairs_vec[i].second;

        //Note: the names of the variables below are not their literal meanings, just for convenience of the code writing

        // check the actual distance between the hand capsule pair
        std::shared_ptr<ComputeLinksDistance> compute_distance =
                std::make_shared<ComputeLinksDistance>(*_model_ptr, this->urdf, this->srdf);
        std::list<std::pair<std::string,std::string> > whiteList_;
        whiteList_.push_back(std::pair<std::string,std::string>(_linkA,_linkB));
        compute_distance->setCollisionWhiteList(whiteList_);
        std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
        LinkPairDistance result = results.front();
        double reference_distance;
        reference_distance = result.getDistance();
        //std::cout<<"reference_distance: "<<reference_distance<<std::endl;


        if (i == 0)
        {
            std::cout << "checking the distance between hands" << std::endl;
            EXPECT_NEAR(0.005, reference_distance, 1e-4);
        }
        else if (i ==1)
        {
            std::cout << "checking the distance between legs" << std::endl;
            EXPECT_NEAR(0.005, reference_distance, 1e-4);
        }
        else
            std::cout << "The dimension of CasulePairs_vec is incorrect!" << std::endl;

    }


    // change whitelist: release the foot pair

    // reset whitelist

    std::cout << "xxx Resetting whitelist" << std::endl;
    whiteList.pop_back();
    this->sc_constraint->setCollisionWhiteList(whiteList);
    std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

    dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq.setZero(dq.size());}
        this->q += dq;
#if ENABLE_ROS
        this->publishJointStates(this->q);
        usleep(10000);
#endif
    }

    std::cout << "Q_final 2: " << this->q.transpose() << std::endl;

    std::cout<<"Initial Left Arm 2: "<<T_init_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left Arm 2: "<<T_reference_l_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left Arm 2: "<<task_left_arm->getActualPose()<<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm 2: "<<T_init_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right Arm 2: "<<T_reference_r_arm<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right Arm 2: "<<task_right_arm->getActualPose()<<std::endl;

    // showing the data of the legs

    std::cout<<std::endl;

    std::cout<<"Initial Left leg 2: "<<T_init_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Left leg 2: "<<T_reference_l_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Left leg 2: "<<task_left_leg->getActualPose()<<std::endl;;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right leg 2: "<<T_init_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Reference Right leg 2: "<<T_reference_r_leg<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Actual Right leg 2: "<<task_right_leg->getActualPose()<<std::endl;

    std::cout<<std::endl;

    // start rechecking the distances of the capsules

    // check the actual distance between the hand capsule pair
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string urdf_capsule_path = robotology_root + "/external/OpenSoT/tests/robots/bigman/bigman_capsules.rviz";
    std::string srdf_capsule_path = robotology_root + "/external/OpenSoT/tests/robots/bigman/bigman.srdf";


    urdf::ModelSharedPtr urdf = MAKE_SHARED<urdf::Model>();
    urdf->initFile(urdf_capsule_path);

    srdf::ModelSharedPtr srdf = MAKE_SHARED<srdf::Model>();
    srdf->initFile(*urdf, srdf_capsule_path);
    std::shared_ptr<ComputeLinksDistance> compute_distance =
            std::make_shared<ComputeLinksDistance>(*_model_ptr, urdf, srdf);
    compute_distance->setCollisionWhiteList(whiteList);
    std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
    LinkPairDistance result = results.front();
    double reference_distance;
    reference_distance = result.getDistance();
    std::cout<<"reference_distance: "<<reference_distance<<std::endl;

    //checking the distance between hands

    EXPECT_NEAR(0.005, reference_distance, 1e-4);

    //checking if actual positions of the feet are coincident with the goal reference positions

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_left_leg->getActualPose()(i,j), T_reference_l_leg(i,j), 1E-4);

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_right_leg->getActualPose()(i,j), T_reference_r_leg(i,j), 1E-4);


}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
