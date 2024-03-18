#include <gtest/gtest.h>
#include <OpenSoT/utils/collision_utils.h>
#include <cmath>
#include <fcl/narrowphase/detail/primitive_shape_algorithm/capsule_capsule-inl.h>
#include <xbot2_interface/xbotinterface2.h>
#include <chrono>

#define ENABLE_ROS false

#if ENABLE_ROS
    #include <ros/ros.h>
    #include <sensor_msgs/JointState.h>
    #include <visualization_msgs/Marker.h>
#endif


#define  _s                1.0
#define  dT               0.001* _s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)
#define SMALL_NUM 1e-5

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

KDL::Frame fcl2KDL(const fcl::Transform3<double> &in)
{
    Eigen::Quaterniond q(in.linear());
    Eigen::Vector3d t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    return f;
}

Eigen::Affine3d fcl2Eigen(const fcl::Transform3<double> &in)
{
    Eigen::Quaterniond q(in.linear());
    Eigen::Vector3d t = in.translation();

    Eigen::Affine3d T;
    T.linear() = q.matrix();
    T.translation() = t;

    return T;
}

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <eigen_conversions/eigen_kdl.h>
// tf::vectorKDLToEigen
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i];
}

void getGoodInitialPosition(Eigen::VectorXd& q, const XBot::ModelInterface::Ptr robot) {

    q[robot->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[robot->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[robot->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q[robot->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[robot->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[robot->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[robot->getQIndex("LShSag")] =  20.0*M_PI/180.0;
    q[robot->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    q[robot->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    q[robot->getQIndex("RShSag")] =  20.0*M_PI/180.0;
    q[robot->getQIndex("RShLat")] = -10.0*M_PI/180.0;
    q[robot->getQIndex("RElbj")] = -80.0*M_PI/180.0;

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


    std::map<std::string,std::shared_ptr<fcl::CollisionObject<double>> > getcollision_objects()
    {
        return _computeDistance.getCollisionObjects();
    }

    std::map<std::string, Eigen::Affine3d> getlink_T_shape()
    {
        return _computeDistance.getLinkToShapeTransforms();
    }

    bool updateCollisionObjects()
    {
        return _computeDistance.updateCollisionObjects();
    }

    bool globalToLinkCoordinates(const std::string& linkName,
                                 const fcl::Transform3<double> &fcl_w_T_f,
                                 Eigen::Affine3d &link_T_f)
    {

        return _computeDistance.globalToLinkCoordinates(linkName, fcl_w_T_f, link_T_f);
    }

    bool globalToLinkCoordinatesE(const std::string& linkName,
                                    const fcl::Transform3<double> &fcl_w_T_f,
                                    Eigen::Affine3d &link_T_f)
    {

        Eigen::Affine3d w_T_f = fcl2Eigen(fcl_w_T_f);

        fcl::Transform3<double> fcl_w_T_shape = _computeDistance.getCollisionObjects()[linkName]->getTransform();
        Eigen::Affine3d w_T_shape = fcl2Eigen(fcl_w_T_shape);

        Eigen::Affine3d shape_T_f = w_T_shape.inverse()*w_T_f;

        link_T_f = _computeDistance.getLinkToShapeTransforms()[linkName] * shape_T_f;

        return true;
    }

};

namespace {

class testCollisionUtils : public ::testing::Test{
public:

protected:

  testCollisionUtils()
  {
      std::string robot_folder = OPENSOT_TEST_PATH "robots/bigman";


      _model_ptr = XBot::ModelInterface::getModel(ReadFile(robot_folder + "/" + "bigman" + ".urdf"),
                                                  ReadFile(robot_folder + "/" + "bigman" + ".srdf"),
                                                  OPENSOT_TEST_MODEL_TYPE);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

      q = _model_ptr->getNeutralQ();


      std::string urdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman_capsules.rviz";
      std::string srdf_capsule_path = OPENSOT_TEST_PATH "robots/bigman/bigman.srdf";


      urdf::ModelSharedPtr urdf = MAKE_SHARED<urdf::Model>();
      urdf->initFile(urdf_capsule_path);

      srdf::ModelSharedPtr srdf = MAKE_SHARED<srdf::Model>();
      srdf->initFile(*urdf, srdf_capsule_path);


      compute_distance.reset(new ComputeLinksDistance(*_model_ptr, urdf, srdf));
  }

  std::string ReadFile(std::string path)
  {
      std::ifstream t(path);
      std::stringstream buffer;
      buffer << t.rdbuf();
      return buffer.str();
  }

  virtual ~testCollisionUtils() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  Eigen::VectorXd q;
  std::shared_ptr<ComputeLinksDistance> compute_distance;
  XBot::ModelInterface::Ptr _model_ptr;

};

TEST_F(testCollisionUtils, testDistanceChecksAreInvariant) {

      std::list<std::pair<std::string,std::string> > whiteList;
      whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
      compute_distance->setCollisionWhiteList(whiteList);

      getGoodInitialPosition(q,_model_ptr);
      _model_ptr->setJointPosition(q);
      _model_ptr->update();

      std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
      LinkPairDistance result1 = results.front();

      _model_ptr->setJointPosition(q);
      _model_ptr->update();
      results.clear();
      results = compute_distance->getLinkDistances();
      LinkPairDistance result2 = results.front();
      ASSERT_EQ(result1.getDistance(), result2.getDistance());
      ASSERT_EQ(result1.getLinkNames(), result2.getLinkNames());
      for(unsigned int i = 0; i < 3; ++i){
        ASSERT_EQ(result1.getClosestPoints().first.translation()[i], result2.getClosestPoints().first.translation()[i]);
        ASSERT_EQ(result1.getClosestPoints().second.translation()[i], result2.getClosestPoints().second.translation()[i]);}

}

TEST_F(testCollisionUtils, testCapsuleDistance) {

    getGoodInitialPosition(q,_model_ptr);

    q[_model_ptr->getQIndex("LHipLat")] = -1.4*M_PI/180.;
    q[_model_ptr->getQIndex("RHipLat")] = 1.4*M_PI/180.;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();



#if ENABLE_ROS
    int argc = 1;
    char *argv[] = {""};
    ros::init(argc, argv, "testCapsuleDistance");

    ros::NodeHandle n;

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    for(unsigned int i = 0; i < _model_ptr->getJointNames().size(); ++i)
    {
        msg.name.push_back(_model_ptr->getJointNames()[i]);
        msg.position.push_back(q[_model_ptr->getQIndex(_model_ptr->getJointNames()[i])]);
    }


    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    ros::Rate rate = 10;
    for(unsigned int i = 0; i <= 100; ++i)
    {
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
#endif



    std::string linkA = "LLowLeg";
    std::string linkB = "RLowLeg";

    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
    compute_distance->setCollisionWhiteList(whiteList);

    std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
    LinkPairDistance result = results.front();
    double actual_distance;
    actual_distance = result.getDistance();
    ASSERT_EQ(result.getLinkNames().first, linkA);
    ASSERT_EQ(result.getLinkNames().second, linkB);


#if ENABLE_ROS
    ros::Publisher pub2 = n.advertise<visualization_msgs::Marker>("link_distances", 1, true);
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
    for(const auto& data : results)
    {
        auto k2p = [](const Eigen::Vector3d &k)->geometry_msgs::Point{
            geometry_msgs::Point p;
            p.x = k[0]; p.y = k[1]; p.z = k[2];
            return p;
        };



        // closest point on first link
        marker.points.push_back(k2p(data.getClosestPoints().first.translation()));
        // closest point on second link
        marker.points.push_back(k2p(data.getClosestPoints().second.translation()));
    }

    for(unsigned int i = 0; i < 100; ++i){
        pub2.publish(marker);
        ros::spinOnce();
        rate.sleep();
    }
#endif





    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);
    std::map<std::string,std::shared_ptr<fcl::CollisionObject<double>> > collision_objects_test;
    std::map<std::string,Eigen::Affine3d> link_T_shape_test;

    collision_objects_test = compute_distance_observer.getcollision_objects();
    link_T_shape_test = compute_distance_observer.getlink_T_shape();

    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_l = collision_objects_test[linkA];
    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_r = collision_objects_test[linkB];


    int left_hand_index = _model_ptr->getLinkId(linkA);
    if(left_hand_index == -1)
        std::cout << "Failed to get lefthand_index" << std::endl;

    int right_hand_index = _model_ptr->getLinkId(linkB);
    if(right_hand_index == -1)
        std::cout << "Failed to get righthand_index" << std::endl;



    Eigen::Affine3d w_T_link_left_hand;
    _model_ptr->getPose(linkA, w_T_link_left_hand);
    Eigen::Affine3d w_T_link_right_hand;
    _model_ptr->getPose(linkB, w_T_link_right_hand);

    double actual_distance_check =
        (   result.getClosestPoints().first.translation() -
            result.getClosestPoints().second.translation()
        ).norm();

    fcl::DistanceRequest<double> distance_request;
#if FCL_MINOR_VERSION > 2
    distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
    distance_request.enable_nearest_points = true;

    fcl::DistanceResult<double> distance_result;

    fcl::CollisionObject<double>* left_hand_collision_object =
        collision_geometry_l.get();
    fcl::CollisionObject<double>* right_hand_collision_object =
        collision_geometry_r.get();

    fcl::distance(left_hand_collision_object, right_hand_collision_object,
                  distance_request,
                  distance_result);
#if FCL_MINOR_VERSION > 2
    double actual_distance_check_original =
        (distance_result.nearest_points[0] - distance_result.nearest_points[1]).norm();
#else
    double actual_distance_check_original =
        (distance_result.nearest_points[0] - distance_result.nearest_points[1]).length();
#endif



    EXPECT_NEAR(actual_distance, actual_distance_check, 1E-8);
    EXPECT_NEAR(actual_distance_check, actual_distance_check_original, 1E-8);

    std::cout<<"actual_distance: "<<actual_distance<<std::endl;
    std::cout<<"actual_distance_check: "<<actual_distance_check<<std::endl;
    std::cout<<"actual_distance_check_original: "<<actual_distance_check_original<<std::endl;
}




TEST_F(testCollisionUtils, checkTimings)
{
    getGoodInitialPosition(q,_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);

    std::map<std::string,std::shared_ptr<fcl::CollisionObject<double>> > collision_objects_test;
    std::map<std::string,Eigen::Affine3d> link_T_shape_test;

    collision_objects_test = compute_distance_observer.getcollision_objects();
    link_T_shape_test = compute_distance_observer.getlink_T_shape();

    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_l = collision_objects_test[linkA];
    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_r = collision_objects_test[linkB];

    int left_hand_index = _model_ptr->getLinkId(linkA);
    if(left_hand_index == -1)
        std::cout << "Failed to get lefthand_index" << std::endl;

    int right_hand_index = _model_ptr->getLinkId(linkB);
    if(right_hand_index == -1)
        std::cout << "Failed to get righthand_index" << std::endl;

    auto tic = std::chrono::high_resolution_clock::now();
    compute_distance_observer.updateCollisionObjects();
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = toc - tic;
    std::cout << "updateCollisionObjects t: " << elapsed.count() << std::endl;

    tic = std::chrono::high_resolution_clock::now();
    compute_distance->getLinkDistances();
    toc = std::chrono::high_resolution_clock::now();
    elapsed = toc - tic;
    std::cout << "getLinkDistances() t: " << elapsed.count() << std::endl;

    tic = std::chrono::high_resolution_clock::now();
    compute_distance->getLinkDistances(0.05);
    toc = std::chrono::high_resolution_clock::now();
    elapsed = toc - tic;
    std::cout << "getLinkDistances(0.05) t: " << elapsed.count() << std::endl;

    {
        tic = std::chrono::high_resolution_clock::now();
        fcl::DistanceRequest<double> distance_request;
#if FCL_MINOR_VERSION > 2
        distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
        distance_request.enable_nearest_points = true;

        fcl::DistanceResult<double> distance_result;

        fcl::CollisionObject<double>* left_hand_collision_object =
            collision_geometry_l.get();
        fcl::CollisionObject<double>* right_hand_collision_object =
            collision_geometry_r.get();

        fcl::distance(left_hand_collision_object, right_hand_collision_object,
                      distance_request,
                      distance_result);
        toc = std::chrono::high_resolution_clock::now();
        elapsed = toc - tic;
        std::cout << "fcl capsule-capsule t: " << elapsed.count() << std::endl;
    }

    {
        tic = std::chrono::high_resolution_clock::now();
        fcl::DistanceRequest<double> distance_request;
#if FCL_MINOR_VERSION > 2
        distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
        distance_request.enable_nearest_points = false;

        fcl::DistanceResult<double> distance_result;

        fcl::CollisionObject<double>* left_hand_collision_object =
            collision_geometry_l.get();
        fcl::CollisionObject<double>* right_hand_collision_object =
            collision_geometry_r.get();

        fcl::distance(left_hand_collision_object, right_hand_collision_object,
                      distance_request,
                      distance_result);

        toc = std::chrono::high_resolution_clock::now();
        elapsed = toc - tic;
        std::cout << "fcl capsule-capsule without closest-point query t: " << elapsed.count() << std::endl;
    }
}

TEST_F(testCollisionUtils, testGlobalToLinkCoordinates)
{
    getGoodInitialPosition(q,_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);

    std::map<std::string,std::shared_ptr<fcl::CollisionObject<double>> > collision_objects_test;
    std::map<std::string,Eigen::Affine3d> link_T_shape_test;

    collision_objects_test = compute_distance_observer.getcollision_objects();
    link_T_shape_test = compute_distance_observer.getlink_T_shape();

    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_l = collision_objects_test[linkA];
    std::shared_ptr<fcl::CollisionObject<double>> collision_geometry_r = collision_objects_test[linkB];

    int left_hand_index = _model_ptr->getLinkId(linkA);
    if(left_hand_index == -1)
        std::cout << "Failed to get lefthand_index" << std::endl;

    int right_hand_index = _model_ptr->getLinkId(linkB);
    if(right_hand_index == -1)
        std::cout << "Failed to get righthand_index" << std::endl;

    auto tic = std::chrono::high_resolution_clock::now();
    compute_distance_observer.updateCollisionObjects();
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = toc - tic;
    std::cout << "updateCollisionObjects t: " << elapsed.count() << std::endl;

    tic = std::chrono::high_resolution_clock::now();
    compute_distance->getLinkDistances();
    toc = std::chrono::high_resolution_clock::now();
    elapsed = toc - tic;
    std::cout << "getLinkDistances() t: " << elapsed.count() << std::endl;

    tic = std::chrono::high_resolution_clock::now();
    compute_distance->getLinkDistances(0.05);
    toc = std::chrono::high_resolution_clock::now();
    elapsed = toc - tic;
    std::cout << "getLinkDistances(0.05) t: " << elapsed.count() << std::endl;


        tic = std::chrono::high_resolution_clock::now();
        fcl::DistanceRequest<double> distance_request;
#if FCL_MINOR_VERSION > 2
        distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
        distance_request.enable_nearest_points = true;

        fcl::DistanceResult<double> distance_result;

        fcl::CollisionObject<double>* left_hand_collision_object =
            collision_geometry_l.get();
        fcl::CollisionObject<double>* right_hand_collision_object =
            collision_geometry_r.get();

        fcl::distance(left_hand_collision_object, right_hand_collision_object,
                      distance_request,
                      distance_result);
        toc = std::chrono::high_resolution_clock::now();
        elapsed = toc - tic;
        std::cout << "fcl capsule-capsule t: " << elapsed.count() << std::endl;


    Eigen::Affine3d lA_T_pA_;
    Eigen::Affine3d lA_T_pA;

    fcl::Transform3<double> T; T.setIdentity();
    T.translation() = distance_result.nearest_points[0];

    compute_distance_observer.globalToLinkCoordinatesE(linkA,T,lA_T_pA_);
    compute_distance_observer.globalToLinkCoordinates(linkA,T,lA_T_pA);

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(lA_T_pA_.translation()[i], lA_T_pA.translation()[i], 1e-9);
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            EXPECT_NEAR(lA_T_pA_.linear()(i,j), lA_T_pA.linear()(i,j), 1e-9);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
