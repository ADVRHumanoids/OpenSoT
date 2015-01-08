#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class testQPOases_sot: public ::testing::Test
{
protected:
    typedef boost::shared_ptr<KDL::Trajectory> TrajPtr;
    typedef boost::shared_ptr<KDL::Path> PathPtr;
    typedef boost::shared_ptr<KDL::VelocityProfile> VelProfPtr;
    typedef boost::shared_ptr<KDL::RotationalInterpolation> RotIntPtr;
    std::ofstream _log;

    RotIntPtr rotationInterpolationMethod;
    PathPtr path;
    VelProfPtr velocityProfile;
    TrajPtr trajectory;

    testQPOases_sot() :
        rotationInterpolationMethod(
            new KDL::RotationalInterpolation_SingleAxis())
    {
        _log.open("testQPOases_FF.m", std::ofstream::app);
    }

    virtual ~testQPOases_sot() {
        _log.close();

        trajectory.reset();
        velocityProfile.reset();
        path.reset();
        rotationInterpolationMethod.reset();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void get5cmFwdLinearTraj(KDL::Frame& start) {

        // 5 cm forward
        KDL::Frame end = start;
        end.p[0] += .05;

        // construct an equivalent radius so that the length
        // of the path along the arc of .1rad will be .05m
        // .05 = (eqRad*.1);
        double eqRad = .05/.1;

        path = PathPtr( new KDL::Path_Line(start, end,
                                           rotationInterpolationMethod.get()->Clone(),
                                           eqRad));

        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.01,.005));
        velocityProfile->SetProfile(0,path->PathLength());
        trajectory = TrajPtr( new KDL::Trajectory_Segment(path.get()->Clone(),
                                                          velocityProfile.get()->Clone()));
    }

    void getdot1radCircularTraj(KDL::Frame& start) {
        // 5 cm forward
        KDL::Frame end = start;
        end.M.DoRotX(.1);

        // construct an equivalent radius so that the length
        // of the path along the arc of .1rad will be .05m
        // .05 = (eqRad*.1);
        double eqRad = .05/.1;
        path = PathPtr( new KDL::Path_Line(start, end,
                                           rotationInterpolationMethod.get()->Clone(),
                                           eqRad));
        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.01,.005));
        velocityProfile->SetProfile(0,path->PathLength());
        trajectory = TrajPtr( new KDL::Trajectory_Segment(path.get()->Clone(),
                                                          velocityProfile.get()->Clone()));
    }
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


TEST_F(testQPOases_sot, testCartesianFF) 
{
#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("testCartesianFF");
#endif

    iDynUtils model;

    yarp::sig::Vector q = getGoodInitialPosition(model);
    model.updateiDyn3Model(q, true);
    model.switchAnchorAndFloatingBase(model.left_leg.end_effector_name);

#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    // BOUNDS

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits(
            new OpenSoT::constraints::velocity::JointLimits( q,
                        model.iDyn3_model.getJointBoundMax(),
                        model.iDyn3_model.getJointBoundMin()));

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsVelocityLimits(
            new OpenSoT::constraints::velocity::VelocityLimits( 0.3,1e-3,q.size()));

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    bounds_list.push_back(boundsVelocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm_task(
                new OpenSoT::tasks::velocity::Cartesian("l_arm",q, model,
                                                        model.left_arm.end_effector_name,
                                                        "world"));
    l_arm_task->setLambda(.6);
    l_arm_task->setOrientationErrorGain(.1);

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    _log.close();
    _log.open("testQPOases_FF.m");

    stack_of_tasks.push_back(l_arm_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds,1e9));



    yarp::sig::Vector dq(q.size(), 0.0);

    double dt=1e-3;

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(1Khz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;
    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist estimated_twist, desired_twist;

    yarp::sig::Matrix current_pose_y = l_arm_task->getActualPose();
    YarptoKDL(current_pose_y,current_pose);
    get5cmFwdLinearTraj(current_pose);
    desired_pose = trajectory->Pos(0.0);
    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    double t_loop = dt;
    double t_compute = 0;
    for (double t=0.0; t <= trajectory->Duration(); t+= t_loop)
    {
        double t_begin = yarp::os::SystemClock::nowSystem();
        yarp::sig::Matrix desired_pose_y(4,0.0);
        yarp::sig::Vector desired_twist_y(6,0.0);
        desired_pose = trajectory->Pos(t);
        desired_twist = trajectory->Vel(t);
        KDLtoYarp_position(desired_pose, desired_pose_y);
        KDLtoYarp(desired_twist, desired_twist_y);
        l_arm_task->setReference(desired_pose_y, desired_twist_y*t_loop);


        model.updateiDyn3Model(q, true);

        l_arm_task->update(q);
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

        current_pose_y = l_arm_task->getActualPose();
        YarptoKDL(current_pose_y,current_pose);


        t_compute = yarp::os::SystemClock::nowSystem() - t_begin;
        yarp::os::SystemClock::delaySystem(dt-t_compute);
        t_loop = yarp::os::SystemClock::nowSystem() - t_begin;

        estimated_twist[0] = (current_pose.p[0] - previous_pose.p[0])/t_loop;

        _log << t << ",\t"
             << estimated_twist[0] << ",\t"
             << desired_twist[0]   << ",\t"
             << current_pose.p[0]  << ",\t"
             << desired_pose.p[0]  << ",\t"
             << t_compute          << ",\t"
             << t_loop             << ";" << std::endl;
        // also velocities and accelerations are available !
        previous_pose = current_pose;
        EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],5e-5);
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,2:3));" << std::endl;
    _log << "legend('Desired Velocity Profile', 'Actual Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Desired Position', 'Actual Position','Location','SouthEast');" << std::endl;


//    yarp::sig::Matrix right_foot_pose = right_foot_task->getActualPose();

//    for(i = 0; i < n_iterations; ++i)
//    {
//        model.updateiDyn3Model(q, true);

//        l_arm_task->update(q);
//        postural_task->update(q);
//        bounds->update(q);

//        _log << com_task->getActualPosition()[0] << ","
//            << com_task->getActualPosition()[1] <<";";
//        e = norm(T_com_p_ref - com_task->getActualPosition());

//        if(fabs(previous_e - e) < 1e-13) {
//            std::cout << "i: " << i << " e: " << norm(T_com_p_ref - com_task->getActualPosition()) << " . Error not decreasing. CONVERGED." << std::endl;
//            break;
//        }
//        previous_e = e;

//        EXPECT_TRUE(sot->solve(dq));
//        q += dq;

//        yarp::sig::Matrix right_foot_pose_now = right_foot_task->getActualPose();
//        for(unsigned int r = 0; r < 4; ++r)
//            for(unsigned int c = 0; c < 4; ++c)
//                ASSERT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-6);

//#ifdef TRY_ON_SIMULATOR
//        robot.move(q);
//#endif
//    }

//    ASSERT_NEAR(norm(T_com_p_ref - com_task->getActualPosition()),0,1E-9);

//    model.updateiDyn3Model(q, true);
//    boundsConvexHull->update(q);
//    std::vector<KDL::Vector> points;
//    std::vector<KDL::Vector> points_inner;
//    boundsConvexHull->getConvexHull(points);

//    KDL::Vector point_old;
//    yarp::sig::Matrix A_ch, A_ch_outer;
//    yarp::sig::Vector b_ch, b_ch_outer;
//    boundsConvexHull->getConstraints(points, A_ch, b_ch, 0.01);
//    boundsConvexHull->getConstraints(points, A_ch_outer, b_ch_outer, 0.0);
//    std::cout << std::endl << "A_ch: " << std::endl << A_ch.toString() << std::endl;
//    std::cout << std::endl << "b_ch: " << std::endl << b_ch.toString() << std::endl;
//    std::cout << std::endl << "A_ch_outer: " << std::endl << A_ch_outer.toString() << std::endl;
//    std::cout << std::endl << "b_ch_outer: " << std::endl << b_ch_outer.toString() << std::endl;
//    std::cout << std::endl << "@q: " << std::endl << q.toString() << std::endl << std::endl;
//    getPointsFromConstraints(A_ch,
//                             b_ch,
//                             points_inner);

//    points.push_back(points.front());
//    points_inner.push_back(points_inner.front());
//    for(KDL::Vector point : points) {
//        std::cout << std::endl
//                  << "=================" << std::endl
//                  << "Moving from ("
//                  << point_old.x() << "," << point_old.y()
//                  << ") to ("
//                  << point.x() << "," << point.y()
//                  <<")" << std::endl;

//        T_com_p_ref[0] = point.x();
//        T_com_p_ref[1] = point.y();

//        com_task->setReference(T_com_p_ref);

//        yarp::sig::Vector dq(q.size(), 0.0);
//        double e = norm(T_com_p_ref - com_task->getActualPosition());
//        double previous_e = 0.0;
//        double oscillation_check_e = 0.0;
//        for(i = 0; i < n_iterations; ++i)
//        {
//            model.updateiDyn3Model(q, true);

//            first_task->update(q);
//            right_foot_task->update(q);
//            com_task->update(q);
//            postural_task->update(q);
//            bounds->update(q);

//            _log << com_task->getActualPosition()[0] << ","
//                << com_task->getActualPosition()[1] <<";";
//            e = norm(T_com_p_ref - com_task->getActualPosition());

//            if(fabs(e - oscillation_check_e) < 1e-9)
//            {
//                std::cout << "i: " << i << " e: " << e << " -- Oscillation detected. Stopping." << std::endl;
//                break;
//            }

//            //std::cout << "i: " << i << " e: " << e << std::endl;
//            if(fabs(previous_e - e) < 1e-12) {
//                std::cout << "i: " << i << " e: " << e << " -- Error not decreasing. CONVERGED." << std::endl;
//                break;
//            }
//            oscillation_check_e = previous_e;
//            previous_e = e;

//            if(e < 1e-3) {  // what if we get too close?!?
//                boundsConvexHull->getConstraints(points, A_ch, b_ch, 0.01);
//                std::cout << "A_ch:" << A_ch.toString() << std::endl;
//                std::cout << "b_ch:" << b_ch.toString() << std::endl;
//            }

//            EXPECT_TRUE(sot->solve(dq));
//            q += dq;

//            yarp::sig::Matrix right_foot_pose_now = right_foot_task->getActualPose();
//            for(unsigned int r = 0; r < 4; ++r)
//                for(unsigned int c = 0; c < 4; ++c)
//                    EXPECT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-3) << "Error at iteration "
//                                                                                    << i
//                                                                                    << " at position "
//                                                                                    << "(" << r
//                                                                                    << "," << c << ")"
//                                                                                    << " with cartesian error equal to "
//                                                                                    << norm(right_foot_task->getb()) << std::endl;
//            EXPECT_LT(norm(right_foot_task->getA()*dq),1E-6) << "Error at iteration "
//                                                             << i
//                                                             << " J_foot*dq = "
//                                                             << (right_foot_task->getA()*dq).toString()
//                                                             << std::endl;

//#ifdef TRY_ON_SIMULATOR
//            robot.move(q);
//#endif
//        }
//        if(i == n_iterations)
//            std::cout << "i: " << i << " e: " << norm(T_com_p_ref - com_task->getActualPosition()) << " -- Error not decreasing. STOPPING." << std::endl;

//        yarp::sig::Vector distance = T_com_p_ref - com_task->getActualPosition();
//        double d = norm(distance);
//        yarp::sig::Vector exp_distance(2,0.01);
//        double expected_d = norm(exp_distance);

//        std::vector<KDL::Vector> points_check;
//        boundsConvexHull->getConvexHull(points_check);
//        yarp::sig::Matrix A_ch_check;
//        yarp::sig::Vector b_ch_check;
//        boundsConvexHull->getConstraints(points_check, A_ch_check, b_ch_check, 0.01);

//        EXPECT_NEAR(d, expected_d, (expected_d-0.01)*1.01) << "Failed to reach point "
//                                                           << point
//                                                           << " in the allocated threshold (0.01m)." << std::endl;
//        EXPECT_TRUE(A_ch == A_ch_check) << "Convex Hull changed!" << std::endl;
//        if(! (A_ch == A_ch_check) )
//        {
//            std::cout << "Old A:" << std::endl << A_ch.toString() << std::endl;
//            std::cout << "New A:" << std::endl << A_ch_check.toString() << std::endl;
//            std::cout << "Had originally " << points.size() -1 << " points in the Convex Hull, "
//                      << " now we have " << points_check.size() << std::endl;
//        }
//        point_old = point;
//    }

//    _log << "];" << std::endl;

//    _log << "points=[";
//    for(KDL::Vector point : points)
//        _log << point.x() << "," << point.y() << ";";
//    _log << "];" << std::endl;
//    _log << "points_inner=[";
//    for(KDL::Vector point : points_inner)
//        _log << point.x() << "," << point.y() << ";";
//    _log << "];" << std::endl;

//    _log << "figure; hold on; plot2(points,'r'); plot2(points_inner,'g'); axis equal;" << std::endl;
//    _log << "ct = plot2(com_traj,'b');" << std::endl;
}

TEST_F(testQPOases_sot, testPosturalFF)
{

}

TEST_F(testQPOases_sot, testCoMFF)
{

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
