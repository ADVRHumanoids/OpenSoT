#include <qpOASES/Matrices.hpp>
#include <gtest/gtest.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <memory>
#include <OpenSoT/Task.h>
#include <yarp/math/Math.h>
#include <qpOASES.hpp>
#include <ctime>
#include <qpOASES/Utils.hpp>
#include <fstream>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>
#include <idynutils/cartesian_utils.h>
#include <qpOASES/Utils.hpp>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <tf/tf.h>

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
// tf::vectorKDLToEigen
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i];
}

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

#define mSecToSec(X) (X*0.001)


#define toRad(X) (X*M_PI/180.0)
#define SMALL_NUM 1e-5

using namespace yarp::sig;
using namespace yarp::math;

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

    std::map<std::string,std::shared_ptr<fcl::CollisionGeometry> > getShapes()
    {
        return _computeDistance.shapes_;
    }

    std::map<std::string,std::shared_ptr<fcl::CollisionObject> > getcollision_objects()
    {
        return _computeDistance.collision_objects_;
    }

    std::map<std::string,KDL::Frame> getlink_T_shape()
    {
        return _computeDistance.link_T_shape;
    }

    std::map<std::string,std::shared_ptr<ComputeLinksDistance::Capsule> > getcustom_capsules()
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

yarp::sig::Vector getGoodInitialPosition2(iDynUtils& idynutils) {
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


namespace
{


    class testSelfCollisionAvoidanceGlobConstr : public ::testing::Test{
     protected:

      testSelfCollisionAvoidanceGlobConstr():
          robot("bigman",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf"),
          q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
          compute_distance(robot),
          sc_constraint(new OpenSoT::constraints::velocity::SelfCollisionAvoidance(
                            cartesian_utils::toEigen(q), robot, std::numeric_limits<double>::infinity(), 0.005))
      {}

      virtual ~testSelfCollisionAvoidanceGlobConstr() {
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

    struct ik_problem
    {
        std::vector<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> stack_of_tasks;
        OpenSoT::constraints::Aggregated::Ptr bounds;
        double damped_least_square_eps;
    };

    TEST_F(testSelfCollisionAvoidanceGlobConstr, testGlobConstr)
    {
        std::vector<double> mean_time_solver;
        std::vector<double> init_time_solver;
            iDynUtils robot_model("coman",
                                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
            robot_model.iDyn3_model.setFloatingBaseLink(robot_model.left_leg.index);
            yarp::sig::Vector state = getGoodInitialPosition(robot_model);
            robot_model.updateiDyn3Model(state, true);

            double dT = 2.0;

            using namespace OpenSoT::constraints::velocity;
            using namespace OpenSoT::tasks::velocity;
            /** Create Constraints **/
                /** 1) Constraint Convex Hull **/
                ConvexHull::Ptr constraintConvexHull(ConvexHull::Ptr(new ConvexHull(
                                                                         cartesian_utils::toEigen(state), robot_model, 0.02)));

                /** 2) CoM Velocity **/
                Eigen::VectorXd tmp(3);
                tmp<<0.9,0.9,0.9;
                CoMVelocity::Ptr constraintCoMVel(CoMVelocity::Ptr(
                                            new CoMVelocity(tmp, mSecToSec(dT),
                                                            cartesian_utils::toEigen(state), robot_model)));

            /** Create tasks **/
                /** 1) Cartesian RSole **/
                Cartesian::Ptr taskRSole(Cartesian::Ptr(new Cartesian("cartesian::r_sole",
                                                                      cartesian_utils::toEigen(state),robot_model,
                                                                  robot_model.right_leg.end_effector_name,"world")));

                /** 2) Cartesian Waist **/
                Cartesian::Ptr taskWaist(Cartesian::Ptr(
                                             new Cartesian("cartesian::Waist",
                                                           cartesian_utils::toEigen(state),robot_model,"Waist","world")));
                yarp::sig::Matrix waistInit = cartesian_utils::fromEigentoYarp(taskWaist->getActualPose());
                yarp::sig::Matrix waistRef = waistInit;
                waistRef(0,3) += 0.07;
                taskWaist->setReference(cartesian_utils::toEigen(waistRef));
                taskWaist->update(cartesian_utils::toEigen(state));

                /** 3) Cartesian Torso **/
                Cartesian::Ptr taskTorso(Cartesian::Ptr(
                                             new Cartesian("cartesian::torso",
                                                           cartesian_utils::toEigen(state),robot_model,"torso","world")));
                    /** 3.1) We want to control torso in /world frame using only the three joints in the torso **/
                    std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
                    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
                        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
                    taskTorso->setActiveJointsMask(active_joint_mask);

                    /** 3.2) We are interested only in the orientation of the torso **/
                    yarp::sig::Matrix W_torso(6,6); W_torso = W_torso.eye();
                    W_torso(0,0) = 0.0; W_torso(1,1) = 0.0; W_torso(2,2) = 0.0;
                    taskTorso->setWeight(cartesian_utils::toEigen(W_torso));
        //            OpenSoT::SubTask::Ptr taskTorsoPosition(OpenSoT::SubTask::Ptr(
        //                                            new OpenSoT::SubTask(taskTorso, OpenSoT::SubTask::SubTaskMap::range(3,5))));
                    Cartesian::Ptr taskRArm(Cartesian::Ptr(new Cartesian("cartesian::r_arm",
                                                                         cartesian_utils::toEigen(state),robot_model,
                                                                      robot_model.right_arm.end_effector_name,"Waist")));
                    Cartesian::Ptr taskLArm(Cartesian::Ptr(new Cartesian("cartesian::l_arm",
                                                                         cartesian_utils::toEigen(state),robot_model,
                                                                      robot_model.left_arm.end_effector_name,"Waist")));

                /** 4) Postural **/
                Postural::Ptr taskPostural(Postural::Ptr(new Postural(cartesian_utils::toEigen(state))));
                //taskPostural->setLambda(0.0);

                /** 5) Mininimize Acceleration **/
                MinimizeAcceleration::Ptr taskMinimizeAcceleration(MinimizeAcceleration::Ptr(
                                                                       new MinimizeAcceleration(
                                                                           cartesian_utils::toEigen(state))));

                /** Create bounds **/
                /** 1) bounds joint limits **/
                JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(
                                                                                           cartesian_utils::toEigen(state),
                                                                    robot_model.getJointBoundMax(),
                                                                    robot_model.getJointBoundMin())));
                /** 2) bounds joint velocities **/
                VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(
                                                            new VelocityLimits(0.1, mSecToSec(dT), state.size())));


                std::shared_ptr<ik_problem> problem(new ik_problem());


                /** Create Augmented (aggregated) tasks  and stack of tasks**/
                /** 1) Higher priority Stack **/
                std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
                taskList.push_back(taskRSole);
                problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                                      new OpenSoT::tasks::Aggregated(taskList, state.size())));
//                /** 1.1) Add constraints to the stack **/
//                problem->stack_of_tasks[0]->getConstraints().push_back(constraintConvexHull);
//                problem->stack_of_tasks[0]->getConstraints().push_back(constraintCoMVel);

                /** 2) Second stack **/
                taskList.clear();
                taskList.push_back(taskWaist);
                taskList.push_back(taskTorso);
                taskList.push_back(taskRArm);
                taskList.push_back(taskLArm);
                problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                                      new OpenSoT::tasks::Aggregated(taskList, state.size())));
//                /** 2.1) Add constraints to the stack **/
//                problem->stack_of_tasks[1]->getConstraints().push_back(constraintConvexHull);
//                problem->stack_of_tasks[1]->getConstraints().push_back(constraintCoMVel);



                /** 3) Third stack **/
                taskList.clear();
                taskList.push_back(taskPostural);
                taskList.push_back(taskMinimizeAcceleration);
                problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                                      new OpenSoT::tasks::Aggregated(taskList, state.size())));
//                /** 3.1) Add constraints to the stack **/
//                problem->stack_of_tasks[2]->getConstraints().push_back(constraintConvexHull);
//                problem->stack_of_tasks[2]->getConstraints().push_back(constraintCoMVel);

                std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> globalConstraintsList;
                globalConstraintsList.push_back(constraintConvexHull);
                globalConstraintsList.push_back(constraintCoMVel);
                OpenSoT::constraints::Aggregated::Ptr globalConstraints(
                            new OpenSoT::constraints::Aggregated(globalConstraintsList,
                                                                 cartesian_utils::toEigen(state)));

                /** Add bounds to problem **/
                std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
                bounds.push_back(boundJointLimits);
                bounds.push_back(boundsJointVelLimits);
                problem->bounds = OpenSoT::constraints::Aggregated::Ptr(
                            new OpenSoT::constraints::Aggregated(bounds, state.size()));

                /** Set damped leas squares fator **/
                problem->damped_least_square_eps = 2E2;

                std::shared_ptr<OpenSoT::solvers::QPOases_sot> qp_solver_sparse;

                int step = 3000;
                    std::cout<<GREEN<<"SPARSE SOLVER"<<DEFAULT<<std::endl;
                    double tic = qpOASES::getCPUtime();
                    qp_solver_sparse = OpenSoT::solvers::QPOases_sot::Ptr(new OpenSoT::solvers::QPOases_sot(
                                                                             problem->stack_of_tasks,
                                                                             problem->bounds, globalConstraints,
                                                                             problem->damped_least_square_eps));
                    double toc = qpOASES::getCPUtime();
                    init_time_solver.push_back(toc-tic);

                    yarp::sig::Vector dq(state.size(), 0.0);
                    Eigen::VectorXd _dq(dq.size()); _dq.setZero(_dq.rows());
                    double acc = 0.0;
                    for(unsigned int i = 0; i < step; ++i)
                    {
                        robot_model.updateiDyn3Model(state, true);

                        for(unsigned int j = 0; j < problem->stack_of_tasks.size(); ++j)
                            problem->stack_of_tasks[j]->update(cartesian_utils::toEigen(state));
                        problem->bounds->update(cartesian_utils::toEigen(state));
                        constraintCoMVel->update(cartesian_utils::toEigen(state));
                        constraintConvexHull->update(cartesian_utils::toEigen(state));

                        double tic = qpOASES::getCPUtime();
                        ASSERT_TRUE(qp_solver_sparse->solve(_dq));
                        double toc = qpOASES::getCPUtime();
                        acc += toc - tic;
                        dq = cartesian_utils::fromEigentoYarp(_dq);
                        state += dq;
                    }
                    double t = acc/(double)(step);
                    std::cout<<"Medium Time to Solve sot "<<acc/(double)(step)<<"[s]"<<std::endl;
                    mean_time_solver.push_back(t);

                    yarp::sig::Matrix waistActual = cartesian_utils::fromEigentoYarp(taskWaist->getActualPose());
                    std::cout<<GREEN<<"Waist Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(waistInit);
                    std::cout<<GREEN<<"Waist Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(waistRef);
                    std::cout<<GREEN<<"Waist Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(waistActual);
                    for(unsigned int ii = 0; ii < 3; ++ii)
                        EXPECT_NEAR(waistActual(ii,3), waistRef(ii,3), 1E-3);
                    for(unsigned int ii = 0; ii < 3; ++ii)
                        for(unsigned int jj = 0; jj < 3; ++jj)
                            EXPECT_NEAR(waistActual(ii,jj), waistRef(ii,jj), 1E-2);

    }

    TEST_F(testSelfCollisionAvoidanceGlobConstr, testMultipleCapsulePairsSC){

        this->robot.iDyn3_model.setFloatingBaseLink(this->robot.left_leg.index);
        this->q = getGoodInitialPosition2(this->robot);
        this->robot.updateiDyn3Model(this->q, true);

        std::string linkA = "LSoftHandLink";
        std::string linkB = "RSoftHandLink";

        std::string linkC = "LFootmot";
        std::string linkD = "RFootmot";

        std::string linkE = "LElb";
        std::string linkF = "RElb";

        std::string linkG = "TorsoProtections";


        // arm task

        OpenSoT::tasks::velocity::Cartesian::Ptr task_left_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist",
                                                            cartesian_utils::toEigen(this->q), this->robot,linkA, "Waist"));
        task_left_arm->setOrientationErrorGain(0.1);

        OpenSoT::tasks::velocity::Cartesian::Ptr task_right_arm(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist",
                                                            cartesian_utils::toEigen(this->q), this->robot,linkB, "Waist"));
        task_right_arm->setOrientationErrorGain(0.1);

        yarp::sig::Matrix T_init_l_arm(4,4);
        T_init_l_arm = cartesian_utils::fromEigentoYarp(task_left_arm->getReference());

        yarp::sig::Matrix T_init_r_arm(4,4);
        T_init_r_arm = cartesian_utils::fromEigentoYarp(task_right_arm->getReference());

        yarp::sig::Matrix T_reference_l_arm(4,4);
        T_reference_l_arm = cartesian_utils::fromEigentoYarp(task_left_arm->getReference());
        T_reference_l_arm(1,3) = 0.0;
        task_left_arm->setReference(cartesian_utils::toEigen(T_reference_l_arm));

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
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_leg",
                                                            cartesian_utils::toEigen(this->q), this->robot,"LFoot", "Waist"));
        task_left_leg->setOrientationErrorGain(0.1);

        OpenSoT::tasks::velocity::Cartesian::Ptr task_right_leg(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_leg",
                                                            cartesian_utils::toEigen(this->q), this->robot,"RFoot", "Waist"));
        task_right_leg->setOrientationErrorGain(0.1);

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
        whiteList.push_back(std::pair<std::string,std::string>(linkE,linkG));
        whiteList.push_back(std::pair<std::string,std::string>(linkF,linkG));
        this->sc_constraint->setCollisionWhiteList(whiteList);
        std::cout << "xxx Whitelist of size " << whiteList.size() << " set. Constraint automatically updated" << std::endl;

        std::list<OpenSoT::tasks::velocity::Cartesian::TaskPtr> cartesianTasks;
        cartesianTasks.push_back(task_left_arm);
        cartesianTasks.push_back(task_right_arm);
        cartesianTasks.push_back(task_left_leg);
        cartesianTasks.push_back(task_right_leg);
        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr taskCartesianAggregated = OpenSoT::tasks::Aggregated::TaskPtr(
                    new OpenSoT::tasks::Aggregated(cartesianTasks,this->q.size()));
        //taskCartesianAggregated->getConstraints().push_back(this->sc_constraint);

        OpenSoT::tasks::velocity::Postural::Ptr postural_task(new OpenSoT::tasks::velocity::Postural(
                                                                  cartesian_utils::toEigen(this->q)));
        //postural_task->getConstraints().push_back(this->sc_constraint);


        OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

        stack_of_tasks.push_back(taskCartesianAggregated);
        stack_of_tasks.push_back(postural_task);

        int t = 100;
        OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
                    new OpenSoT::constraints::velocity::JointLimits(cartesian_utils::toEigen(this->q),
                                                                    this->robot.getJointBoundMax(),
                                                                    this->robot.getJointBoundMin()));

        OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.6, (double)(1.0/t), this->q.size()));

        OpenSoT::constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_limits, joint_velocity_limits, this->q.size()));

        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> globalConstraintsList;
        globalConstraintsList.push_back(this->sc_constraint);
        OpenSoT::constraints::Aggregated::Ptr globalConstraints(
                    new OpenSoT::constraints::Aggregated(globalConstraintsList, cartesian_utils::toEigen(this->q)));

        OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr sot = OpenSoT::solvers::QPOases_sot::Ptr(
                    new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds, globalConstraints));

        yarp::sig::Vector dq(this->q.size(), 0.0);
        Eigen::VectorXd _dq(dq.size()); _dq.setZero(dq.size());
        for(unsigned int i = 0; i < 50*t; ++i)
        {
            this->robot.updateiDyn3Model(this->q, true);


            taskCartesianAggregated->update(cartesian_utils::toEigen(this->q));
            postural_task->update(cartesian_utils::toEigen(this->q));
            bounds->update(cartesian_utils::toEigen(this->q));
            globalConstraints->update(cartesian_utils::toEigen(this->q));

            if(!sot->solve(_dq)){
                std::cout<<"error"<<std::endl;
                _dq.setZero(dq.size());}
            dq = cartesian_utils::fromEigentoYarp(_dq);
            this->q += dq;

        }

        std::cout << "Q_final: " << q.toString() << std::endl;

        std::cout<<"Initial Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    T_init_l_arm);
        std::cout<<std::endl;
        std::cout<<"Reference Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    T_reference_l_arm);
        std::cout<<std::endl;
        std::cout<<"Actual Left Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(task_left_arm->getActualPose()));

        std::cout<<std::endl;
        std::cout<<std::endl;

        std::cout<<"Initial Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    T_init_r_arm);
        std::cout<<std::endl;
        std::cout<<"Reference Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(T_reference_r_arm));
        std::cout<<std::endl;
        std::cout<<"Actual Right Arm: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(task_right_arm->getActualPose()));

        // showing the data of the legs

        std::cout<<std::endl;

        std::cout<<"Initial Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(T_init_l_leg));
        std::cout<<std::endl;
        std::cout<<"Reference Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(T_reference_l_leg));
        std::cout<<std::endl;
        std::cout<<"Actual Left leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(task_left_leg->getActualPose()));

        std::cout<<std::endl;
        std::cout<<std::endl;

        std::cout<<"Initial Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(T_init_r_leg));
        std::cout<<std::endl;
        std::cout<<"Reference Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(T_reference_r_leg));
        std::cout<<std::endl;
        std::cout<<"Actual Right leg: "<<std::endl; cartesian_utils::printHomogeneousTransform(
                    cartesian_utils::fromEigentoYarp(task_right_leg->getActualPose()));

        std::cout<<std::endl;

        // start checking the distances of the capsules

        typedef std::pair<std::string,std::string> CapsulePair;
        std::vector<CapsulePair> CasulePairs_vec;
        CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkA,linkB));
        CasulePairs_vec.push_back(std::pair<std::string,std::string>(linkC,linkD));


        TestCapsuleLinksDistance compute_distance_observer(compute_distance);

        int i;
        for (i=0; i < CasulePairs_vec.size(); i++)
        {

            std::string _linkA = CasulePairs_vec[i].first;
            std::string _linkB = CasulePairs_vec[i].second;

            //Note: the names of the variables below are not their literal meanings, just for convenience of the code writing

            KDL::Vector lefthand_capsule_ep1, lefthand_capsule_ep2,
                    righthand_capsule_ep1, righthand_capsule_ep2;

            std::shared_ptr<ComputeLinksDistance::Capsule> capsuleA = compute_distance_observer.getcustom_capsules()[_linkA];
            std::shared_ptr<ComputeLinksDistance::Capsule> capsuleB = compute_distance_observer.getcustom_capsules()[_linkB];
            capsuleA->getEndPoints(lefthand_capsule_ep1, lefthand_capsule_ep2);
            capsuleB->getEndPoints(righthand_capsule_ep1, righthand_capsule_ep2);

            int left_wrist_index = robot.iDyn3_model.getLinkIndex(_linkA);
            if(left_wrist_index == -1)
                std::cout << "Failed to get index" << std::endl;

            int right_wrist_index = robot.iDyn3_model.getLinkIndex(_linkB);
            if(right_wrist_index == -1)
                std::cout << "Failed to get index" << std::endl;

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
    }



}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
