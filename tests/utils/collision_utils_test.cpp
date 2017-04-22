#include <gtest/gtest.h>
#include <OpenSoT/utils/collision_utils.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <cmath>
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>

#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)
#define SMALL_NUM 1e-5

KDL::Frame fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Quaternion3f q = in.getQuatRotation();
    fcl::Vec3f t = in.getTranslation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());

    return f;
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

void getGoodInitialPosition(Eigen::VectorXd& q, const idynutils2& robot) {

    q[robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    q[robot.iDynTree_model.getDOFIndex("RShSag")] =  20.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("RShLat")] = -10.0*M_PI/180.0;
    q[robot.iDynTree_model.getDOFIndex("RElbj")] = -80.0*M_PI/180.0;

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

    std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > getShapes()
    {
        return _computeDistance.shapes_;
    }

    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > getcollision_objects()
    {
        return _computeDistance.collision_objects_;
    }

    std::map<std::string,KDL::Frame> getlink_T_shape()
    {
        return _computeDistance.link_T_shape;
    }

    std::map<std::string,boost::shared_ptr<ComputeLinksDistance::Capsule> > getcustom_capsules()
    {
        return _computeDistance.custom_capsules_;
    }

    bool updateCollisionObjects()
    {
        return _computeDistance.updateCollisionObjects();
    }

    bool globalToLinkCoordinates(const std::string& linkName,
                                 const fcl::Transform3f &fcl_w_T_f,
                                 KDL::Frame &link_T_f)
    {

        return _computeDistance.globalToLinkCoordinates(linkName, fcl_w_T_f, link_T_f);
    }

    bool globalToLinkCoordinatesKDL(const std::string& linkName,
                                    const fcl::Transform3f &fcl_w_T_f,
                                    KDL::Frame &link_T_f)
    {

        KDL::Frame w_T_f = fcl2KDL(fcl_w_T_f);

        fcl::Transform3f fcl_w_T_shape = _computeDistance.collision_objects_[linkName]->getTransform();
        KDL::Frame w_T_shape = fcl2KDL(fcl_w_T_shape);

        KDL::Frame shape_T_f = w_T_shape.Inverse()*w_T_f;

        link_T_f = _computeDistance.link_T_shape[linkName] * shape_T_f;

        return true;
    }

};

namespace {

class testCollisionUtils : public ::testing::Test{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}

protected:

  testCollisionUtils():
      robot("bigman",
            std::string(std::getenv("ROBOTOLOGY_ROOT"))+"/external/OpenSoT/tests/robots/bigman/bigman.urdf",
            std::string(std::getenv("ROBOTOLOGY_ROOT"))+"/external/OpenSoT/tests/robots/bigman/bigman.srdf")
  {
      q.setZero(robot.iDynTree_model.getNrOfDOFs());


      std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
      std::string relative_path = "/external/OpenSoT/tests/configs/bigman/configs/config_bigman.yaml";

      _path_to_cfg = robotology_root + relative_path;

      _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
              (XBot::ModelInterface::getModel(_path_to_cfg));
      _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&robot, &null_deleter));

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

      compute_distance.reset(new ComputeLinksDistance(*_model_ptr));
  }

  virtual ~testCollisionUtils() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils robot;
  Eigen::VectorXd q;
  boost::shared_ptr<ComputeLinksDistance> compute_distance;
  XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
  std::string _path_to_cfg;


};

TEST_F(testCollisionUtils, testDistanceChecksAreInvariant) {

      std::list<std::pair<std::string,std::string> > whiteList;
      whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
      compute_distance->setCollisionWhiteList(whiteList);

      getGoodInitialPosition(q,robot);
      robot.updateiDynTreeModel(q, false);

      std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
      LinkPairDistance result1 = results.front();

      robot.updateiDynTreeModel(q, false);
      results.clear();
      results = compute_distance->getLinkDistances();
      LinkPairDistance result2 = results.front();
      ASSERT_EQ(result1.getDistance(), result2.getDistance());
      ASSERT_EQ(result1.getLinkNames(), result2.getLinkNames());
      ASSERT_EQ(result1.getLink_T_closestPoint(), result2.getLink_T_closestPoint());

}

TEST_F(testCollisionUtils, testCapsuleDistance) {

    getGoodInitialPosition(q,robot);
    robot.updateiDynTreeModel(q, false);

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    std::list<std::pair<std::string,std::string> > whiteList;
    whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
    compute_distance->setCollisionWhiteList(whiteList);

    std::list<LinkPairDistance> results = compute_distance->getLinkDistances();
    LinkPairDistance result = results.front();
    double actual_distance;
    actual_distance = result.getDistance();
    ASSERT_EQ(result.getLinkNames().first, linkA);
    ASSERT_EQ(result.getLinkNames().second, linkB);

    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);
    std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > shapes_test;
    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > collision_objects_test;
    std::map<std::string,KDL::Frame> link_T_shape_test;

    shapes_test = compute_distance_observer.getShapes();
    collision_objects_test = compute_distance_observer.getcollision_objects();
    link_T_shape_test = compute_distance_observer.getlink_T_shape();

    boost::shared_ptr<fcl::CollisionObject> collision_geometry_l = collision_objects_test[linkA];
    boost::shared_ptr<fcl::CollisionObject> collision_geometry_r = collision_objects_test[linkB];

    int left_hand_index = robot.iDynTree_model.getLinkIndex(linkA);
    if(left_hand_index == -1)
        std::cout << "Failed to get lefthand_index" << std::endl;

    int right_hand_index = robot.iDynTree_model.getLinkIndex(linkB);
    if(right_hand_index == -1)
        std::cout << "Failed to get righthand_index" << std::endl;

    KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_hand_index);
    KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_hand_index);

    double actual_distance_check =
        (   ( w_T_link_left_hand *
              result.getLink_T_closestPoint().first ).p -
            ( w_T_link_right_hand *
              result.getLink_T_closestPoint().second ).p
        ).Norm();

    fcl::DistanceRequest distance_request;
#if FCL_MINOR_VERSION > 2
    distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
    distance_request.enable_nearest_points = true;

    fcl::DistanceResult distance_result;

    fcl::CollisionObject* left_hand_collision_object =
        collision_geometry_l.get();
    fcl::CollisionObject* right_hand_collision_object =
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

    KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                righthand_capsule_ep1, righthand_capsule_ep2;

    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[linkA];
    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[linkB];
    capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
    capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);
    lefthand_capsule_ep1 = w_T_link_left_hand * lefthand_capsule_ep1;
    lefthand_capsule_ep2 = w_T_link_left_hand * lefthand_capsule_ep2;
    righthand_capsule_ep1 = w_T_link_right_hand * righthand_capsule_ep1;
    righthand_capsule_ep2 = w_T_link_right_hand * righthand_capsule_ep2;

    Eigen::Vector3d lefthand_capsule_ep1_eigen, lefthand_capsule_ep2_eigen,
                    righthand_capsule_ep1_eigen, righthand_capsule_ep2_eigen;

    Eigen::Vector3d lefthand_CP, righthand_CP;
    double reference_distance;

    vectorKDLToEigen(lefthand_capsule_ep1, lefthand_capsule_ep1_eigen);
    vectorKDLToEigen(lefthand_capsule_ep2, lefthand_capsule_ep2_eigen);
    vectorKDLToEigen(righthand_capsule_ep1, righthand_capsule_ep1_eigen);
    vectorKDLToEigen(righthand_capsule_ep2, righthand_capsule_ep2_eigen);

    reference_distance = dist3D_Segment_to_Segment (lefthand_capsule_ep1_eigen,
                                                    lefthand_capsule_ep2_eigen,
                                                    righthand_capsule_ep1_eigen,
                                                    righthand_capsule_ep2_eigen,
                                                    lefthand_CP,
                                                    righthand_CP);

    reference_distance = reference_distance
        - capsuleA->getRadius()
        - capsuleB->getRadius();
    double reference_distance_check = (lefthand_CP - righthand_CP).norm()
        - capsuleA->getRadius()
        - capsuleB->getRadius();


    // we compute the distance by knowing the two hands are parallel (but not the capsules!) and the capsules have the same radii
    double hand_computed_distance_estimate = (w_T_link_left_hand.p - w_T_link_right_hand.p).Norm()
        - capsuleA->getRadius()
        - capsuleB->getRadius();

    EXPECT_NEAR(actual_distance, actual_distance_check, 1E-8);
    EXPECT_NEAR(actual_distance_check, actual_distance_check_original, 1E-8);
    EXPECT_NEAR(reference_distance, reference_distance_check, 1E-8);
    EXPECT_NEAR(actual_distance, reference_distance, 1E-4) << "estimate was " << hand_computed_distance_estimate;

}

TEST_F(testCollisionUtils, checkTimings)
{
    getGoodInitialPosition(q,robot);
    robot.updateiDynTreeModel(q, false);

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);

    std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > shapes_test;
    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > collision_objects_test;
    std::map<std::string,KDL::Frame> link_T_shape_test;

    shapes_test = compute_distance_observer.getShapes();
    collision_objects_test = compute_distance_observer.getcollision_objects();
    link_T_shape_test = compute_distance_observer.getlink_T_shape();

    boost::shared_ptr<fcl::CollisionObject> collision_geometry_l = collision_objects_test[linkA];
    boost::shared_ptr<fcl::CollisionObject> collision_geometry_r = collision_objects_test[linkB];

    int left_hand_index = robot.iDynTree_model.getLinkIndex(linkA);
    if(left_hand_index == -1)
        std::cout << "Failed to get lefthand_index" << std::endl;

    int right_hand_index = robot.iDynTree_model.getLinkIndex(linkB);
    if(right_hand_index == -1)
        std::cout << "Failed to get righthand_index" << std::endl;

    double tic = yarp::os::SystemClock::nowSystem();
    compute_distance_observer.updateCollisionObjects();
    std::cout << "updateCollisionObjects t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;

    tic = yarp::os::SystemClock::nowSystem();
    compute_distance->getLinkDistances();
    std::cout << "getLinkDistances() t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;

    tic = yarp::os::SystemClock::nowSystem();
    compute_distance->getLinkDistances(0.05);
    std::cout << "getLinkDistances(0.05) t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;

    {
        tic = yarp::os::SystemClock::nowSystem();
        fcl::DistanceRequest distance_request;
#if FCL_MINOR_VERSION > 2
        distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
        distance_request.enable_nearest_points = true;

        fcl::DistanceResult distance_result;

        fcl::CollisionObject* left_hand_collision_object =
            collision_geometry_l.get();
        fcl::CollisionObject* right_hand_collision_object =
            collision_geometry_r.get();

        fcl::distance(left_hand_collision_object, right_hand_collision_object,
                      distance_request,
                      distance_result);
        std::cout << "fcl capsule-capsule t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;
    }

    {
        tic = yarp::os::SystemClock::nowSystem();
        fcl::DistanceRequest distance_request;
#if FCL_MINOR_VERSION > 2
        distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
        distance_request.enable_nearest_points = false;

        fcl::DistanceResult distance_result;

        fcl::CollisionObject* left_hand_collision_object =
            collision_geometry_l.get();
        fcl::CollisionObject* right_hand_collision_object =
            collision_geometry_r.get();

        fcl::distance(left_hand_collision_object, right_hand_collision_object,
                      distance_request,
                      distance_result);
        std::cout << "fcl capsule-capsule without closest-point query t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;
    }

    tic = yarp::os::SystemClock::nowSystem();
    KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                righthand_capsule_ep1, righthand_capsule_ep2;

    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[linkA];
    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[linkB];

    capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
    capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

    Eigen::Vector3d lefthand_capsule_ep1_eigen, lefthand_capsule_ep2_eigen,
                    righthand_capsule_ep1_eigen, righthand_capsule_ep2_eigen;

    Eigen::Vector3d lefthand_CP, righthand_CP;
    double reference_distance;

    vectorKDLToEigen(lefthand_capsule_ep1, lefthand_capsule_ep1_eigen);
    vectorKDLToEigen(lefthand_capsule_ep2, lefthand_capsule_ep2_eigen);
    vectorKDLToEigen(righthand_capsule_ep1, righthand_capsule_ep1_eigen);
    vectorKDLToEigen(righthand_capsule_ep2, righthand_capsule_ep2_eigen);

    reference_distance = dist3D_Segment_to_Segment (lefthand_capsule_ep1_eigen,
                                                    lefthand_capsule_ep2_eigen,
                                                    righthand_capsule_ep1_eigen,
                                                    righthand_capsule_ep2_eigen,
                                                    lefthand_CP,
                                                    righthand_CP);
    std::cout << "inline capsule-capsule t: " << yarp::os::SystemClock::nowSystem() - tic << std::endl;
}

TEST_F(testCollisionUtils, testGlobalToLinkCoordinates)
{
    getGoodInitialPosition(q,robot);
    robot.updateiDynTreeModel(q, false);
    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    TestCapsuleLinksDistance compute_distance_observer(*compute_distance);
    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > collision_objects_test =
        compute_distance_observer.getcollision_objects();
    boost::shared_ptr<fcl::CollisionObject> collision_geometry_l = collision_objects_test[linkA];
    boost::shared_ptr<fcl::CollisionObject> collision_geometry_r = collision_objects_test[linkB];

    fcl::DistanceRequest distance_request;
#if FCL_MINOR_VERSION > 2
    distance_request.gjk_solver_type = fcl::GST_INDEP;
#endif
    distance_request.enable_nearest_points = true;

    fcl::DistanceResult distance_result;

    fcl::CollisionObject* left_hand_collision_object =
        collision_geometry_l.get();
    fcl::CollisionObject* right_hand_collision_object =
        collision_geometry_r.get();

    fcl::distance(left_hand_collision_object, right_hand_collision_object,
                  distance_request,
                  distance_result);

    KDL::Frame lA_T_pA_KDL;
    KDL::Frame lA_T_pA;
    compute_distance_observer.globalToLinkCoordinatesKDL(linkA,distance_result.nearest_points[0],lA_T_pA_KDL);
    compute_distance_observer.globalToLinkCoordinates(linkA,distance_result.nearest_points[0],lA_T_pA);

    EXPECT_EQ(lA_T_pA_KDL, lA_T_pA);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
