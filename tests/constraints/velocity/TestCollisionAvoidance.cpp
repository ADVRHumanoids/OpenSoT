#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/solvers/QPOases.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <XBotInterface/ModelInterface.h>
#include <advr_humanoids_common_utils/test_utils.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/all.h>
#include <cmath>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>

#define ENABLE_ROS false

#if ENABLE_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#endif



typedef idynutils2 iDynUtils;

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

KDL::Frame fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Quaternion3f q = in.getQuatRotation();
    fcl::Vec3f t = in.getTranslation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());

    return f;
}

yarp::sig::Vector getGoodInitialPosition(iDynUtils& _robot) {
    yarp::sig::Vector q(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LShYaw")] = -15.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    q[_robot.iDynTree_model.getDOFIndex("RShSag")] =  20.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RShLat")] = -10.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RShYaw")] = 15.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("RElbj")] = -80.0*M_PI/180.0;


    std::cout << "Q_initial: " << q.toString() << std::endl;
    return q;
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

namespace{


class testSelfCollisionAvoidanceConstraint : public ::testing::Test{
public:
    static void null_deleter(iDynUtils *) {}

#if ENABLE_ROS
    void publishJointStates(const Eigen::VectorXd& q)
    {
        for(unsigned int i = 0; i < q.size(); ++i)
            joint_state.position[i] = q[i];
        joint_state.header.stamp = ros::Time::now();

        pub.publish(joint_state);
        ros::spinOnce();
    }
#endif

 protected:

  testSelfCollisionAvoidanceConstraint():
      robot("bigman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf"),
      q(robot.iDynTree_model.getNrOfDOFs(), 0.0)
  {
#if ENABLE_ROS
      int argc = 0;
      char **argv;
      ros::init(argc, argv, "collision_avoidance_test");
      n.reset(new ros::NodeHandle());
      pub = n->advertise<sensor_msgs::JointState>("joint_states", 1000);
#endif

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

      compute_distance.reset(new ComputeLinksDistance(*(_model_ptr.get())));

      double padding = 0.005;//0.005;
      sc_constraint.reset(new OpenSoT::constraints::velocity::SelfCollisionAvoidance(
                        conversion_utils_YARP::toEigen(q), *(_model_ptr.get()),
                              std::numeric_limits<double>::infinity(), padding));

#if ENABLE_ROS
      for(unsigned int i = 0; i < robot.getJointNames().size(); ++i){
          joint_state.name.push_back(robot.getJointNames()[i]);
          joint_state.position.push_back(0.0);}
#endif
  }

  virtual ~testSelfCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }



  XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
  std::string _path_to_cfg;
  iDynUtils robot;
  yarp::sig::Vector q;
  boost::shared_ptr<ComputeLinksDistance> compute_distance;
  OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr sc_constraint;

#if ENABLE_ROS
  ///ROS
  boost::shared_ptr<ros::NodeHandle> n;
  ros::Publisher pub;
  sensor_msgs::JointState joint_state;
#endif


};


TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithoutSC){

    int idx = robot.iDynTree_model.getLinkIndex("l_sole");
    this->robot.iDynTree_model.setFloatingBaseLink(idx);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_hand",
                                                        conversion_utils_YARP::toEigen(this->q), *(_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_hand",
                                                        conversion_utils_YARP::toEigen(this->q), *(_model_ptr.get()), linkB, "Waist"));
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
                new OpenSoT::tasks::velocity::Postural(conversion_utils_YARP::toEigen(this->q)));


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 10;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(conversion_utils_YARP::toEigen(this->q),
                                                        this->robot.getJointBoundMax(),
                                                        this->robot.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);

        taskCartesianAggregated->update(conversion_utils_YARP::toEigen(this->q));
        postural_task->update(conversion_utils_YARP::toEigen(this->q));
        bounds->update(conversion_utils_YARP::toEigen(this->q));

        if(!sot->solve(_dq)){
            std::cout<<"error"<<std::endl;
            _dq.setZero(_dq.rows());}
        dq = conversion_utils_YARP::toYARP(_dq);
        using namespace yarp::math;
        this->q += dq;

    }

    std::cout << "Q_final: " << this->q.toString() << std::endl;

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

    int left_wrist_index = robot.iDynTree_model.getLinkIndex(linkA);
    if(left_wrist_index == -1)
        std::cout << "Failed to get leftwrist_index" << std::endl;

    int right_wrist_index = robot.iDynTree_model.getLinkIndex(linkB);
    if(right_wrist_index == -1)
        std::cout << "Failed to get rightwrist_index" << std::endl;

    KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_wrist_index);
    KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_wrist_index);

    double actual_distance = ( w_T_link_left_hand.p - w_T_link_right_hand.p ).Norm();

    EXPECT_NEAR(0.0, actual_distance, 1E-4);

}


TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithSC){

    int idx = robot.iDynTree_model.getLinkIndex("l_sole");
    this->robot.iDynTree_model.setFloatingBaseLink(idx);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);


    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_hand",
                                                        conversion_utils_YARP::toEigen(this->q), *(_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_hand",
                                                        conversion_utils_YARP::toEigen(this->q), *(_model_ptr.get()), linkB, "Waist"));
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

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(
                                                              conversion_utils_YARP::toEigen(this->q)));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(
                    conversion_utils_YARP::toEigen(this->q),
                                            this->robot.getJointBoundMax(),
                                            this->robot.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);

        double tic = yarp::os::SystemClock::nowSystem();
        this->sc_constraint->update(
                    conversion_utils_YARP::toEigen(this->q));
        std::cout << "Update time:" << yarp::os::SystemClock::nowSystem() - tic << std::endl;

        taskCartesianAggregated->update(
                    conversion_utils_YARP::toEigen(this->q));
        postural_task->update(conversion_utils_YARP::toEigen(this->q));
        bounds->update(conversion_utils_YARP::toEigen(this->q));

        if(!sot->solve(_dq)){
            std::cout<<"error"<<std::endl;
            _dq.setZero(dq.size());}
        dq = conversion_utils_YARP::toYARP(_dq);
        using namespace yarp::math;
        this->q += dq;

    }

    std::cout << "Q_final: " << this->q.toString() << std::endl;

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

    TestCapsuleLinksDistance compute_distance_observer(*(compute_distance.get()));

    KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
            righthand_capsule_ep1, righthand_capsule_ep2;

    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[linkA];
    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[linkB];
    capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
    capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

    int left_wrist_index = robot.iDynTree_model.getLinkIndex(linkA);
    if(left_wrist_index == -1)
        std::cout << "Failed to get leftwrist_index" << std::endl;

    int right_wrist_index = robot.iDynTree_model.getLinkIndex(linkB);
    if(right_wrist_index == -1)
        std::cout << "Failed to get rightwrist_index" << std::endl;

    KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_wrist_index);
    KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_wrist_index);

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

    reference_distance = reference_distance - capsuleA->getRadius() - capsuleB->getRadius();


    EXPECT_NEAR(0.005, reference_distance, 1e-4);


}


TEST_F(testSelfCollisionAvoidanceConstraint, testMultipleCapsulePairsSC){

    int idx = robot.iDynTree_model.getLinkIndex("l_sole");
    this->robot.iDynTree_model.setFloatingBaseLink(idx);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);

#if ENABLE_ROS
    this->publishJointStates(conversion_utils_YARP::toEigen(this->q));
    this->publishJointStates(conversion_utils_YARP::toEigen(this->q));
    this->publishJointStates(conversion_utils_YARP::toEigen(this->q));


    sleep(1);
#endif

    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    std::string linkC = "LFootmot";
    std::string linkD = "RFootmot";

    // arm task
    Eigen::VectorXd q = conversion_utils_YARP::toEigen(this->q);
    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, *(_model_ptr.get()),linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", q, *(_model_ptr.get()),linkB, "Waist"));
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
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg", q, *(_model_ptr.get()),"LFoot", "Waist"));
    task_left_leg->setOrientationErrorGain(0.1);

    Eigen::MatrixXd W = task_left_leg->getWeight();
    //W(1,1) = 1.1;
    W(0,0) = 3.;
    W(2,2) = 3.;
    task_left_leg->setWeight(W);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg", q, *(_model_ptr.get()),"RFoot", "Waist"));
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

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(q));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(q,
                                                                this->robot.getJointBoundMax(),
                                                                this->robot.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);


        taskCartesianAggregated->update(conversion_utils_YARP::toEigen(this->q));
        postural_task->update(conversion_utils_YARP::toEigen(this->q));
        bounds->update(conversion_utils_YARP::toEigen(this->q));

        if(!sot->solve(_dq)){
            std::cout<<"error"<<std::endl;
            _dq.setZero(dq.size());}
        dq = conversion_utils_YARP::toYARP(_dq);
        using namespace yarp::math;
        this->q += dq;
#if ENABLE_ROS
        this->publishJointStates(conversion_utils_YARP::toEigen(this->q));
        usleep(0);
#endif
    }

    std::cout << "Q_final: " << this->q.toString() << std::endl;

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

        KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                righthand_capsule_ep1, righthand_capsule_ep2;

        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[_linkA];
        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[_linkB];
        capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
        capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

        int left_wrist_index = robot.iDynTree_model.getLinkIndex(_linkA);
        if(left_wrist_index == -1)
            std::cout << "Failed to get index" << std::endl;

        int right_wrist_index = robot.iDynTree_model.getLinkIndex(_linkB);
        if(right_wrist_index == -1)
            std::cout << "Failed to get index" << std::endl;

        KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_wrist_index);
        KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_wrist_index);

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

        reference_distance = reference_distance - capsuleA->getRadius() - capsuleB->getRadius();

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

    int idx = robot.iDynTree_model.getLinkIndex("l_sole");
    this->robot.iDynTree_model.setFloatingBaseLink(idx);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);


    std::string linkA = "LSoftHandLink";
    std::string linkB = "RSoftHandLink";

    std::string linkC = "LFootmot";
    std::string linkD = "RFootmot";

    // arm task
    Eigen::VectorXd q = conversion_utils_YARP::toEigen(this->q);
    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, *(this->_model_ptr.get()), linkA, "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", q, *(this->_model_ptr.get()), linkB, "Waist"));
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
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg", q, *(this->_model_ptr.get()),"LFoot", "Waist"));
    task_left_leg->setOrientationErrorGain(0.1);
    Eigen::MatrixXd W = task_left_leg->getWeight();
    //W(1,1) = 1.1;
    W(0,0) = 3.;
    W(2,2) = 3.;
    task_left_leg->setWeight(W);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg", q, *(this->_model_ptr.get()),"RFoot", "Waist"));
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

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(q));
    postural_task->getConstraints().push_back(this->sc_constraint);


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                new OpenSoT::constraints::velocity::JointLimits(q,
                                                                this->robot.getJointBoundMax(),
                                                                this->robot.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);


        taskCartesianAggregated->update(conversion_utils_YARP::toEigen(this->q));
        postural_task->update(conversion_utils_YARP::toEigen(this->q));
        bounds->update(conversion_utils_YARP::toEigen(this->q));

        if(!sot->solve(_dq)){
            std::cout<<"error"<<std::endl;
            _dq.setZero(dq.size());}
        dq = conversion_utils_YARP::toYARP(_dq);
        using namespace yarp::math;
        this->q += dq;

    }

    std::cout << "Q_final: " << this->q.toString() << std::endl;

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

        KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                righthand_capsule_ep1, righthand_capsule_ep2;

        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[_linkA];
        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[_linkB];
        capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
        capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

        int left_wrist_index = robot.iDynTree_model.getLinkIndex(_linkA);
        if(left_wrist_index == -1)
            std::cout << "Failed to get leftwrist_index" << std::endl;

        int right_wrist_index = robot.iDynTree_model.getLinkIndex(_linkB);
        if(right_wrist_index == -1)
            std::cout << "Failed to get rightwrist_index" << std::endl;

        KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_wrist_index);
        KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_wrist_index);

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

        reference_distance = reference_distance - capsuleA->getRadius() - capsuleB->getRadius();


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

    _dq.setZero(dq.size());
    for(unsigned int i = 0; i < 50*t; ++i)
    {
        this->robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(this->q), true);


        taskCartesianAggregated->update(conversion_utils_YARP::toEigen(this->q));
        postural_task->update(conversion_utils_YARP::toEigen(this->q));
        bounds->update(conversion_utils_YARP::toEigen(this->q));

        if(!sot->solve(_dq)){
            std::cout<<"error"<<std::endl;
            _dq.setZero(dq.size());}
        dq = conversion_utils_YARP::toYARP(_dq);
        using namespace yarp::math;
        this->q += dq;

    }

    std::cout << "Q_final 2: " << this->q.toString() << std::endl;

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

    std::string _linkA = CasulePairs_vec[0].first;
    std::string _linkB = CasulePairs_vec[0].second;

    KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
            righthand_capsule_ep1, righthand_capsule_ep2;

    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[_linkA];
    boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[_linkB];
    capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
    capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

    int left_wrist_index = robot.iDynTree_model.getLinkIndex(_linkA);
    if(left_wrist_index == -1)
        std::cout << "Failed to get leftwrist_index" << std::endl;

    int right_wrist_index = robot.iDynTree_model.getLinkIndex(_linkB);
    if(right_wrist_index == -1)
        std::cout << "Failed to get rightwrist_index" << std::endl;

    KDL::Frame w_T_link_left_hand = robot.iDynTree_model.getPositionKDL(left_wrist_index);
    KDL::Frame w_T_link_right_hand = robot.iDynTree_model.getPositionKDL(right_wrist_index);

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

    reference_distance = reference_distance - capsuleA->getRadius() - capsuleB->getRadius();

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
