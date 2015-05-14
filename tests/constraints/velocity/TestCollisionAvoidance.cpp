#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <idynutils/idynutils.h>
#include <idynutils/collision_utils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/all.h>
#include <cmath>
#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>
#include <eigen_conversions/eigen_kdl.h>

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
 protected:

  testSelfCollisionAvoidanceConstraint():
      robot("bigman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf"),
      q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
      compute_distance(robot),
      sc_constraint(new OpenSoT::constraints::velocity::SelfCollisionAvoidance(q, robot, std::numeric_limits<double>::infinity(), 0.005))
  {}

  virtual ~testSelfCollisionAvoidanceConstraint() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  iDynUtils robot;
  yarp::sig::Vector q;
  ComputeLinksDistance compute_distance;
  OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr sc_constraint;

};


  TEST_F(testSelfCollisionAvoidanceConstraint, testConversions) {

    yarp::sig::Matrix testMatrix(6,6);
    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            testMatrix(i,j) = i*j+1;

    Eigen::MatrixXd testEigenMatrix = sc_constraint->from_yarp_to_Eigen_matrix(testMatrix);
    yarp::sig::Matrix resultMatrix = sc_constraint->from_Eigen_to_Yarp_matrix(testEigenMatrix);

    for(unsigned int i = 0; i < testMatrix.rows(); ++i)
        for(unsigned int j = 0; j < testMatrix.cols(); ++j)
            EXPECT_DOUBLE_EQ(testMatrix(i,j), resultMatrix(i,j));

  }

  yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
      yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
      yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
      leg[2] = -25.0 * M_PI/180.0;
      leg[3] =  50.0 * M_PI/180.0;
      leg[4] = -25.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
      idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
      yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
      arm[0] = 20.0 * M_PI/180.0;
      arm[1] = 10.0 * M_PI/180.0;
      arm[2] = -15.0 * M_PI/180.0;
      arm[3] = -80.0 * M_PI/180.0;
      idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
      arm[1] = -arm[1];
      arm[2] = -arm[2];
      idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);

      std::cout << "Q_initial: " << q.toString() << std::endl;
      return q;
  }

  TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithoutSC){

    this->robot.iDyn3_model.setFloatingBaseLink(this->robot.left_leg.index);
    this->q = getGoodInitialPosition(this->robot);
    this->robot.updateiDyn3Model(this->q, true);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, this->robot,"LWrMot3", "Waist"));
    task_left_arm->setOrientationErrorGain(0.1);

    OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, this->robot,"RWrMot3", "Waist"));
    task_right_arm->setOrientationErrorGain(0.1);

    yarp::sig::Matrix T_init_l_arm(4,4);
    T_init_l_arm = task_left_arm->getReference();

    yarp::sig::Matrix T_init_r_arm(4,4);
    T_init_r_arm = task_right_arm->getReference();

    yarp::sig::Matrix T_reference_l_arm(4,4);
    T_reference_l_arm = task_left_arm->getReference();
    T_reference_l_arm(1,3) = 0.0;
    task_left_arm->setReference(T_reference_l_arm);

    yarp::sig::Matrix T_reference_r_arm(4,4);
    T_reference_r_arm = task_right_arm->getReference();
    T_reference_r_arm(1,3) = 0.0;
    task_right_arm->setReference(T_reference_r_arm);

    std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
    cartesianTasks.push_back(task_left_arm);
    cartesianTasks.push_back(task_right_arm);
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
       new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));

    OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));


    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(taskCartesianAggregated);
    stack_of_tasks.push_back(postural_task);

    int t = 100;
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(this->q,
                                                        this->robot.iDyn3_model.getJointBoundMax(),
                                                        this->robot.iDyn3_model.getJointBoundMin()));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

    OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

    OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

    yarp::sig::Vector dq(this->q.size(), 0.0);
    for(unsigned int i = 0; i < 100*t; ++i)
    {
        this->robot.updateiDyn3Model(this->q, true);

        taskCartesianAggregated->update(this->q);
        postural_task->update(this->q);
        bounds->update(this->q);

        if(!sot->solve(dq)){
            std::cout<<"error"<<std::endl;
            dq = 0.0;}
        this->q += dq;

    }

    std::cout << "Q_final: " << q.toString() << std::endl;

    std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_arm->getActualPose());

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_arm);
        std::cout<<std::endl;
    std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_arm);
        std::cout<<std::endl;
    std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_arm->getActualPose());

    std::cout<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_left_arm->getActualPose()(i,j), T_reference_l_arm(i,j), 1E-4);

    for(unsigned int i = 0; i < 4; ++i)
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_NEAR(task_right_arm->getActualPose()(i,j), T_reference_r_arm(i,j), 1E-4);

    // check the distance

    std::string linkA = "LWrMot3";
    std::string linkB = "RWrMot3";

    int left_wrist_index = robot.iDyn3_model.getLinkIndex(linkA);
    if(left_wrist_index == -1)
        std::cout << "Failed to get leftwrist_index" << std::endl;

    int right_wrist_index = robot.iDyn3_model.getLinkIndex(linkB);
    if(right_wrist_index == -1)
        std::cout << "Failed to get rightwrist_index" << std::endl;

    KDL::Frame w_T_link_left_hand = robot.iDyn3_model.getPositionKDL(left_wrist_index);
    KDL::Frame w_T_link_right_hand = robot.iDyn3_model.getPositionKDL(right_wrist_index);

    double actual_distance = ( w_T_link_left_hand.p - w_T_link_right_hand.p ).Norm();

    EXPECT_NEAR(0.0, actual_distance, 1E-4);

  }

    //Now we add the constraint
    TEST_F(testSelfCollisionAvoidanceConstraint, testCartesianTaskWithSC){

        this->robot.iDyn3_model.setFloatingBaseLink(this->robot.left_leg.index);
        this->q = getGoodInitialPosition(this->robot);
        this->robot.updateiDyn3Model(this->q, true);


        std::string linkA = "LSoftHandLink";
        std::string linkB = "RSoftHandLink";

        OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, this->robot,linkA, "Waist"));
        task_left_arm->setOrientationErrorGain(0.1);

        OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, this->robot,linkB, "Waist"));
        task_right_arm->setOrientationErrorGain(0.1);

        yarp::sig::Matrix T_init_l_arm(4,4);
        T_init_l_arm = task_left_arm->getReference();

        yarp::sig::Matrix T_init_r_arm(4,4);
        T_init_r_arm = task_right_arm->getReference();

        yarp::sig::Matrix T_reference_l_arm(4,4);
        T_reference_l_arm = task_left_arm->getReference();
        T_reference_l_arm(1,3) = 0.0;
        task_left_arm->setReference(T_reference_l_arm);

        yarp::sig::Matrix T_reference_r_arm(4,4);
        T_reference_r_arm = task_right_arm->getReference();
        T_reference_r_arm(1,3) = 0.0;
        task_right_arm->setReference(T_reference_r_arm);

        std::cout << "xxx Setting whitelist" << std::endl;
        std::list<std::pair<std::string,std::string>> whiteList;
        whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
        this->sc_constraint->setCollisionWhiteList(whiteList);
        std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

        std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
        cartesianTasks.push_back(task_left_arm);
        cartesianTasks.push_back(task_right_arm);
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
           new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
        taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

        OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));
        postural_task->getConstraints().push_back(this->sc_constraint);


        OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

        stack_of_tasks.push_back(taskCartesianAggregated);
        stack_of_tasks.push_back(postural_task);

        int t = 100;
        OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
            new OpenSoT::constraints::velocity::JointLimits(this->q,
                                                            this->robot.iDyn3_model.getJointBoundMax(),
                                                            this->robot.iDyn3_model.getJointBoundMin()));

        OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

        OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
            new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

        yarp::sig::Vector dq(this->q.size(), 0.0);
        for(unsigned int i = 0; i < 100*t; ++i)
        {
            this->robot.updateiDyn3Model(this->q, true);

            double tic = yarp::os::SystemClock::nowSystem();
            this->sc_constraint->update(this->q);
            std::cout << "Update time:" << yarp::os::SystemClock::nowSystem() - tic << std::endl;

            taskCartesianAggregated->update(this->q);
            postural_task->update(this->q);
            bounds->update(this->q);

            if(!sot->solve(dq)){
                std::cout<<"error"<<std::endl;
                dq = 0.0;}
            this->q += dq;

        }

        std::cout << "Q_final: " << q.toString() << std::endl;

        std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_arm);
            std::cout<<std::endl;
        std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_arm);
            std::cout<<std::endl;
        std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_arm->getActualPose());

        std::cout<<std::endl;
        std::cout<<std::endl;

        std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_arm);
            std::cout<<std::endl;
        std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_arm);
            std::cout<<std::endl;
        std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_arm->getActualPose());

        std::cout<<std::endl;


        TestCapsuleLinksDistance compute_distance_observer(compute_distance);

        KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                    righthand_capsule_ep1, righthand_capsule_ep2;

        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[linkA];
        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[linkB];
        capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
        capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

        int left_wrist_index = robot.iDyn3_model.getLinkIndex(linkA);
        if(left_wrist_index == -1)
            std::cout << "Failed to get leftwrist_index" << std::endl;

        int right_wrist_index = robot.iDyn3_model.getLinkIndex(linkB);
        if(right_wrist_index == -1)
            std::cout << "Failed to get rightwrist_index" << std::endl;

        KDL::Frame w_T_link_left_hand = robot.iDyn3_model.getPositionKDL(left_wrist_index);
        KDL::Frame w_T_link_right_hand = robot.iDyn3_model.getPositionKDL(right_wrist_index);

        lefthand_capsule_ep1 = w_T_link_left_hand * lefthand_capsule_ep1;
        lefthand_capsule_ep2 = w_T_link_left_hand * lefthand_capsule_ep2;
        righthand_capsule_ep1 = w_T_link_right_hand * righthand_capsule_ep1;
        righthand_capsule_ep2 = w_T_link_right_hand * righthand_capsule_ep2;

        Eigen::Vector3d lefthand_capsule_ep1_eigen, lefthand_capsule_ep2_eigen,
                        righthand_capsule_ep1_eigen, righthand_capsule_ep2_eigen;

        Eigen::Vector3d lefthand_CP, righthand_CP;
        double reference_distance;

        tf::vectorKDLToEigen(lefthand_capsule_ep1, lefthand_capsule_ep1_eigen);
        tf::vectorKDLToEigen(lefthand_capsule_ep2, lefthand_capsule_ep2_eigen);
        tf::vectorKDLToEigen(righthand_capsule_ep1, righthand_capsule_ep1_eigen);
        tf::vectorKDLToEigen(righthand_capsule_ep2, righthand_capsule_ep2_eigen);

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

        this->robot.iDyn3_model.setFloatingBaseLink(this->robot.left_leg.index);
        this->q = getGoodInitialPosition(this->robot);
        this->robot.updateiDyn3Model(this->q, true);


        std::string linkA = "LSoftHandLink";
        std::string linkB = "RSoftHandLink";

        std::string linkC = "LLowLeg";
        std::string linkD = "RLowLeg";

        // arm task

        OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", this->q, this->robot,linkA, "Waist"));
        task_left_arm->setOrientationErrorGain(0.1);

        OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", this->q, this->robot,linkB, "Waist"));
        task_right_arm->setOrientationErrorGain(0.1);

        yarp::sig::Matrix T_init_l_arm(4,4);
        T_init_l_arm = task_left_arm->getReference();

        yarp::sig::Matrix T_init_r_arm(4,4);
        T_init_r_arm = task_right_arm->getReference();

        yarp::sig::Matrix T_reference_l_arm(4,4);
        T_reference_l_arm = task_left_arm->getReference();
        T_reference_l_arm(1,3) = 0.0;
        task_left_arm->setReference(T_reference_l_arm);

        yarp::sig::Matrix T_reference_r_arm(4,4);
        T_reference_r_arm = task_right_arm->getReference();
        T_reference_r_arm(1,3) = 0.0;
        task_right_arm->setReference(T_reference_r_arm);

        // leg task

        OpenSoT::tasks::velocity::Cartesian::Ptr task_left_leg(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg", this->q, this->robot,linkC, "Waist"));
        task_left_leg->setOrientationErrorGain(0.1);

        OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg", this->q, this->robot,linkD, "Waist"));
        task_right_leg->setOrientationErrorGain(0.1);

        yarp::sig::Matrix T_init_l_leg(4,4);
        T_init_l_leg = task_left_leg->getReference();

        yarp::sig::Matrix T_init_r_leg(4,4);
        T_init_r_leg = task_right_leg->getReference();

        yarp::sig::Matrix T_reference_l_leg(4,4);
        T_reference_l_leg = task_left_leg->getReference();
        T_reference_l_leg(1,3) = 0.0;
        task_left_leg->setReference(T_reference_l_leg);

        yarp::sig::Matrix T_reference_r_leg(4,4);
        T_reference_r_leg = task_right_leg->getReference();
        T_reference_r_leg(1,3) = 0.0;
        task_right_leg->setReference(T_reference_r_leg);

        // set whitelist

//        std::cout << "xxx Setting whitelist" << std::endl;
//        std::list<std::pair<std::string,std::string>> whiteList;
//        whiteList.push_back(std::pair<std::string,std::string>(linkA,linkB));
//        whiteList.push_back(std::pair<std::string,std::string>(linkC,linkD));
//        this->sc_constraint->setCollisionWhiteList(whiteList);
//        std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

        std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
        cartesianTasks.push_back(task_left_arm);
        cartesianTasks.push_back(task_right_arm);
        cartesianTasks.push_back(task_left_leg);
        cartesianTasks.push_back(task_right_leg);
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
           new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
//        taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

        OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(this->q));
//        postural_task->getConstraints().push_back(this->sc_constraint);


        OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

        stack_of_tasks.push_back(taskCartesianAggregated);
        stack_of_tasks.push_back(postural_task);

        int t = 100;
        OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
            new OpenSoT::constraints::velocity::JointLimits(this->q,
                                                            this->robot.iDyn3_model.getJointBoundMax(),
                                                            this->robot.iDyn3_model.getJointBoundMin()));

        OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

        OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

        OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
            new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds));

        yarp::sig::Vector dq(this->q.size(), 0.0);
        for(unsigned int i = 0; i < 100*t; ++i)
        {
            this->robot.updateiDyn3Model(this->q, true);


            taskCartesianAggregated->update(this->q);
            postural_task->update(this->q);
            bounds->update(this->q);

            if(!sot->solve(dq)){
                std::cout<<"error"<<std::endl;
                dq = 0.0;}
            this->q += dq;

        }

        std::cout << "Q_final: " << q.toString() << std::endl;

        std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_arm);
            std::cout<<std::endl;
        std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_arm);
            std::cout<<std::endl;
        std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_arm->getActualPose());

        std::cout<<std::endl;
        std::cout<<std::endl;

        std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_arm);
            std::cout<<std::endl;
        std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_arm);
            std::cout<<std::endl;
        std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_arm->getActualPose());

        // showing the data of the legs

        std::cout<<std::endl;

        std::cout<<"Initial Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_l_leg);
            std::cout<<std::endl;
        std::cout<<"Reference Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_l_leg);
            std::cout<<std::endl;
        std::cout<<"Actual Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_left_leg->getActualPose());

        std::cout<<std::endl;
        std::cout<<std::endl;

        std::cout<<"Initial Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_init_r_leg);
            std::cout<<std::endl;
        std::cout<<"Reference Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(T_reference_r_leg);
            std::cout<<std::endl;
        std::cout<<"Actual Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(task_right_leg->getActualPose());

        std::cout<<std::endl;

        // start checking the distances of the capsules

        typedef std::pair<std::string,std::string> CapsulePair;
        std::vector<CapsulePair> CasulePairs_vec;
        CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkA,linkB));
        CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkC,linkD));


        TestCapsuleLinksDistance compute_distance_observer(compute_distance);

        for (int i=0; i < CasulePairs_vec.size(); i++)
        {

        std::string linkA = CasulePairs_vec[i].first;
        std::string linkB = CasulePairs_vec[i].second;

        KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                    righthand_capsule_ep1, righthand_capsule_ep2;

        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[linkA];
        boost::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[linkB];
        capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
        capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

        int left_wrist_index = robot.iDyn3_model.getLinkIndex(linkA);
        if(left_wrist_index == -1)
            std::cout << "Failed to get leftwrist_index" << std::endl;

        int right_wrist_index = robot.iDyn3_model.getLinkIndex(linkB);
        if(right_wrist_index == -1)
            std::cout << "Failed to get rightwrist_index" << std::endl;

        KDL::Frame w_T_link_left_hand = robot.iDyn3_model.getPositionKDL(left_wrist_index);
        KDL::Frame w_T_link_right_hand = robot.iDyn3_model.getPositionKDL(right_wrist_index);

        lefthand_capsule_ep1 = w_T_link_left_hand * lefthand_capsule_ep1;
        lefthand_capsule_ep2 = w_T_link_left_hand * lefthand_capsule_ep2;
        righthand_capsule_ep1 = w_T_link_right_hand * righthand_capsule_ep1;
        righthand_capsule_ep2 = w_T_link_right_hand * righthand_capsule_ep2;

        Eigen::Vector3d lefthand_capsule_ep1_eigen, lefthand_capsule_ep2_eigen,
                        righthand_capsule_ep1_eigen, righthand_capsule_ep2_eigen;

        Eigen::Vector3d lefthand_CP, righthand_CP;
        double reference_distance;

        tf::vectorKDLToEigen(lefthand_capsule_ep1, lefthand_capsule_ep1_eigen);
        tf::vectorKDLToEigen(lefthand_capsule_ep2, lefthand_capsule_ep2_eigen);
        tf::vectorKDLToEigen(righthand_capsule_ep1, righthand_capsule_ep1_eigen);
        tf::vectorKDLToEigen(righthand_capsule_ep2, righthand_capsule_ep2_eigen);

        reference_distance = dist3D_Segment_to_Segment (lefthand_capsule_ep1_eigen,
                                                        lefthand_capsule_ep2_eigen,
                                                        righthand_capsule_ep1_eigen,
                                                        righthand_capsule_ep2_eigen,
                                                        lefthand_CP,
                                                        righthand_CP);

        reference_distance = reference_distance - capsuleA->getRadius() - capsuleB->getRadius();


        EXPECT_NEAR(0.005, reference_distance, 1e-4);

        }


  }



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
