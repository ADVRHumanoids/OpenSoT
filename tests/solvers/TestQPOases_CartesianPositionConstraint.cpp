#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_cyclic_closed.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
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

class testQPOases_CartesianPositionConstraint:
        public ::testing::Test
{
protected:
    typedef boost::shared_ptr<KDL::Trajectory> TrajPtr;
    typedef boost::shared_ptr<KDL::Path_RoundedComposite> PathPtr;
    typedef boost::shared_ptr<KDL::VelocityProfile> VelProfPtr;
    typedef boost::shared_ptr<KDL::RotationalInterpolation> RotIntPtr;
    std::ofstream _log;

    RotIntPtr rotationInterpolationMethod;
    PathPtr path;
    VelProfPtr velocityProfile;
    TrajPtr trajectory;

    testQPOases_CartesianPositionConstraint() :
        rotationInterpolationMethod(
            new KDL::RotationalInterpolation_SingleAxis())
    {
        _log.open("testQPOases_CartesianPositionConstraint.m");
    }

    virtual ~testQPOases_CartesianPositionConstraint() {
        _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void getCircularTraj(KDL::Frame& start, const double radius) {
        // 1rad clocwise rotation
        KDL::Frame end = start;
        KDL::Frame center = start; center.p.y(start.p.y() - radius);

        try {
            path = PathPtr( new KDL::Path_RoundedComposite(radius/12, radius/12,
                                                           rotationInterpolationMethod.get()->Clone()) );
        } catch (...) { std::cout << "Error creating the path!" << std::endl; }

        const double n_steps = 12;
        double start_angle = 0;
        unsigned int i;
        try {
            for(i = 1; i < n_steps; ++i)
            {
                double step = i*2.0*M_PI/n_steps;
                KDL::Frame intermediate = center;
                intermediate.p = center.p+KDL::Vector(0,
                                                      radius*cos(start_angle+step),
                                                      radius*sin(start_angle+step));
                path->Add(intermediate);
            }
        } catch(KDL::Error_MotionPlanning_Not_Feasible e) {
            std::cout << "Error adding waypoints "
                      << i << " to path: "
                      << e.GetType() << std::endl;
            assert("Error");
        }

        try {
            path->Finish();
        } catch(...) { std::cout << "Error finalizing path" << std::endl; }

        velocityProfile = VelProfPtr( new KDL::VelocityProfile_Trap(.5,.1));
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

//#define TRY_ON_SIMULATOR
TEST_F(testQPOases_CartesianPositionConstraint, tryFollowingBounds) {

    KDL::Frame start;
    try {
        start.p.y(0.05);
        this->getCircularTraj(start, .05);
    } catch(...) { std::cout << "Error while initializing trajectory!" << std::endl; }

    try {
        double t_loop = 0.0;
        for (double t=0.0; t <= trajectory->Duration(); t+= t_loop)
        {
            double t_begin = yarp::os::SystemClock::nowSystem();
            KDL::Frame desired_pose = trajectory->Pos(t);
            KDL::Twist desired_twist = trajectory->Vel(t);

            std::cout << desired_pose.p << std::endl;

            ASSERT_TRUE(desired_pose.M.GetRot() == start.M.GetRot());

            t_loop = yarp::os::SystemClock::nowSystem() - t_begin;
        }
    } catch(...) { std::cout << "Unknown error!" << std::endl; }


#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("hitTheBounds");
#endif

    iDynUtils idynutils("coman",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
/*
    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);
    idynutils.switchAnchorAndFloatingBase(idynutils.left_leg.end_effector_name);

#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    // BOUNDS

    OpenSoT::constraints::Aggregated::ConstraintPtr boundsJointLimits(
            new OpenSoT::constraints::velocity::JointLimits( q,
                        idynutils.iDyn3_model.getJointBoundMax(),
                        idynutils.iDyn3_model.getJointBoundMin()));

    OpenSoT::constraints::Aggregated::ConstraintPtr velocityLimits(
            new OpenSoT::constraints::velocity::VelocityLimits(0.3,
                                                               3e-3,
                                                               q.size()));


    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds_list;
    bounds_list.push_back(boundsJointLimits);
    // NOTE without this things don't work!
    bounds_list.push_back(velocityLimits);

    OpenSoT::constraints::Aggregated::Ptr bounds(
                new OpenSoT::constraints::Aggregated(bounds_list, q.size()));

    OpenSoT::tasks::velocity::CoM::Ptr com_task(
                new OpenSoT::tasks::velocity::CoM(q, idynutils));
    com_task->setLambda(.6);

    yarp::sig::Matrix W(3,3);
    W.eye(); W(2,2) = .1;
    com_task->setWeight(W);

    OpenSoT::constraints::velocity::CoMVelocity::ConstraintPtr boundsCoMVelocity(
                new OpenSoT::constraints::velocity::CoMVelocity(
                    yarp::sig::Vector(3, 0.05), 0.004 , q, idynutils));
    com_task->getConstraints().push_back(boundsCoMVelocity);
    OpenSoT::constraints::velocity::CartesianPositionConstraint::Ptr boundsCartesianPositionConstraint(
                new OpenSoT::constraints::velocity::CartesianPositionConstraint(q, idynutils, 0.01));
    com_task->getConstraints().push_back(boundsCartesianPositionConstraint);

    OpenSoT::tasks::velocity::Cartesian::Ptr right_foot_task(
                new OpenSoT::tasks::velocity::Cartesian("world::right_foot",
                                                        q, idynutils,
                                                        idynutils.right_leg.end_effector_name,
                                                        "world"));
    right_foot_task->setLambda(.2);
    right_foot_task->setOrientationErrorGain(.1);

    OpenSoT::tasks::Aggregated::Ptr first_task(
                new OpenSoT::tasks::Aggregated( com_task,
                                                right_foot_task,
                                                q.size()));

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));;

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    if(footStrategy == USE_TASK)
    {
        _log.close();
        _log.open("testQPOases_CartesianPositionConstraint.m");

        stack_of_tasks.push_back(right_foot_task);
    }
    else if (footStrategy == USE_CONSTRAINT)
    {
        using namespace OpenSoT::constraints;
        TaskToConstraint::Ptr right_foot_constraint(new TaskToConstraint(right_foot_task));

        com_task->getConstraints().push_back(right_foot_constraint);
        postural_task->getConstraints().push_back(right_foot_constraint);

    }

    stack_of_tasks.push_back(com_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks,
                                          bounds));

    //SET SOME REFERENCES
    yarp::sig::Vector T_com_p_init = idynutils.iDyn3_model.getCOM();
    yarp::sig::Vector T_com_p_ref = T_com_p_init;
    T_com_p_ref[0] = 0.0;
    T_com_p_ref[1] = 0.0;

    std::cout << "Initial CoM position is " << T_com_p_init.toString() << std::endl;
    std::cout << "Moving to (0,0)" << std::endl;

    com_task->setReference(T_com_p_ref);

    yarp::sig::Vector dq(q.size(), 0.0);
    double e = norm(T_com_p_ref - com_task->getActualPosition());
    double previous_e = 0.0;

    unsigned int i = 0;
    const unsigned int n_iterations = 10000;
    if(footStrategy == USE_TASK)
        _log << "com_traj = [";
    else if(footStrategy == USE_CONSTRAINT)
        _log << "com_traj_constraint = [";

    yarp::sig::Matrix right_foot_pose = right_foot_task->getActualPose();

    for(i = 0; i < n_iterations; ++i)
    {
        idynutils.updateiDyn3Model(q, true);

        right_foot_task->update(q);
        com_task->update(q);
        postural_task->update(q);
        bounds->update(q);

        _log << com_task->getActualPosition()[0] << ","
            << com_task->getActualPosition()[1] <<";";
        e = norm(T_com_p_ref - com_task->getActualPosition());

        if(fabs(previous_e - e) < 1e-13) {
            std::cout << "i: " << i << " e: " << norm(T_com_p_ref - com_task->getActualPosition()) << " . Error not decreasing. CONVERGED." << std::endl;
            break;
        }
        previous_e = e;

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

        yarp::sig::Matrix right_foot_pose_now = right_foot_task->getActualPose();
        for(unsigned int r = 0; r < 4; ++r)
            for(unsigned int c = 0; c < 4; ++c)
                ASSERT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-6);
        ASSERT_LT(norm(right_foot_task->getb()),1e-8);

#ifdef TRY_ON_SIMULATOR
        robot.move(q);
#endif
    }

    ASSERT_NEAR(norm(T_com_p_ref - com_task->getActualPosition()),0,1E-9);

    idynutils.updateiDyn3Model(q, true);
    boundsCartesianPositionConstraint->update(q);
    std::vector<KDL::Vector> points;
    std::vector<KDL::Vector> points_inner;
    boundsCartesianPositionConstraint->getCartesianPositionConstraint(points);

    KDL::Vector point_old;
    yarp::sig::Matrix A_ch, A_ch_outer;
    yarp::sig::Vector b_ch, b_ch_outer;
    boundsCartesianPositionConstraint->getConstraints(points, A_ch, b_ch, 0.01);
    boundsCartesianPositionConstraint->getConstraints(points, A_ch_outer, b_ch_outer, 0.0);
    std::cout << std::endl << "A_ch: " << std::endl << A_ch.toString() << std::endl;
    std::cout << std::endl << "b_ch: " << std::endl << b_ch.toString() << std::endl;
    std::cout << std::endl << "A_ch_outer: " << std::endl << A_ch_outer.toString() << std::endl;
    std::cout << std::endl << "b_ch_outer: " << std::endl << b_ch_outer.toString() << std::endl;
    std::cout << std::endl << "@q: " << std::endl << q.toString() << std::endl << std::endl;
    getPointsFromConstraints(A_ch,
                             b_ch,
                             points_inner);

    points.push_back(points.front());
    points_inner.push_back(points_inner.front());
    for(KDL::Vector point : points) {
        std::cout << std::endl
                  << "=================" << std::endl
                  << "Moving from ("
                  << point_old.x() << "," << point_old.y()
                  << ") to ("
                  << point.x() << "," << point.y()
                  <<")" << std::endl;

        T_com_p_ref[0] = point.x();
        T_com_p_ref[1] = point.y();

        com_task->setReference(T_com_p_ref);

        yarp::sig::Vector dq(q.size(), 0.0);
        double e = norm(T_com_p_ref - com_task->getActualPosition());
        double previous_e = 0.0;
        double oscillation_check_e = 0.0;
        for(i = 0; i < n_iterations; ++i)
        {
            idynutils.updateiDyn3Model(q, true);

            first_task->update(q);
            right_foot_task->update(q);
            com_task->update(q);
            postural_task->update(q);
            bounds->update(q);

            _log << com_task->getActualPosition()[0] << ","
                << com_task->getActualPosition()[1] <<";";
            e = norm(T_com_p_ref - com_task->getActualPosition());

            if(fabs(e - oscillation_check_e) < 1e-9)
            {
                std::cout << "i: " << i << " e: " << e << " -- Oscillation detected. Stopping." << std::endl;
                break;
            }

            //std::cout << "i: " << i << " e: " << e << std::endl;
            if(fabs(previous_e - e) < 1e-12) {
                std::cout << "i: " << i << " e: " << e << " -- Error not decreasing. CONVERGED." << std::endl;
                break;
            }
            oscillation_check_e = previous_e;
            previous_e = e;

            if(e < 1e-3) {  // what if we get too close?!?
                boundsCartesianPositionConstraint->getConstraints(points, A_ch, b_ch, 0.01);
                std::cout << "A_ch:" << A_ch.toString() << std::endl;
                std::cout << "b_ch:" << b_ch.toString() << std::endl;
            }

            EXPECT_TRUE(sot->solve(dq));
            q += dq;

            yarp::sig::Matrix right_foot_pose_now = right_foot_task->getActualPose();
            for(unsigned int r = 0; r < 4; ++r)
                for(unsigned int c = 0; c < 4; ++c)
                    EXPECT_NEAR(right_foot_pose(r,c),right_foot_pose_now(r,c),1e-3) << "Error at iteration "
                                                                                    << i
                                                                                    << " at position "
                                                                                    << "(" << r
                                                                                    << "," << c << ")"
                                                                                    << " with cartesian error equal to "
                                                                                    << norm(right_foot_task->getb()) << std::endl;
            EXPECT_LT(norm(right_foot_task->getb()),5e-5);
            EXPECT_LT(norm(right_foot_task->getA()*dq),1E-6) << "Error at iteration "
                                                             << i
                                                             << " J_foot*dq = "
                                                             << (right_foot_task->getA()*dq).toString()
                                                             << std::endl;

#ifdef TRY_ON_SIMULATOR
            robot.move(q);
#endif
        }
        if(i == n_iterations)
            std::cout << "i: " << i << " e: " << norm(T_com_p_ref - com_task->getActualPosition()) << " -- Error not decreasing. STOPPING." << std::endl;

        yarp::sig::Vector distance = T_com_p_ref - com_task->getActualPosition();
        double d = norm(distance);
        yarp::sig::Vector exp_distance(2,0.01);
        double expected_d = norm(exp_distance);

        std::vector<KDL::Vector> points_check;
        boundsCartesianPositionConstraint->getCartesianPositionConstraint(points_check);
        yarp::sig::Matrix A_ch_check;
        yarp::sig::Vector b_ch_check;
        boundsCartesianPositionConstraint->getConstraints(points_check, A_ch_check, b_ch_check, 0.01);

        EXPECT_NEAR(d, expected_d, (expected_d-0.01)*1.01) << "Failed to reach point "
                                                           << point
                                                           << " in the allocated threshold (0.01m)." << std::endl;
        //EXPECT_TRUE(A_ch == A_ch_check) << "Convex Hull changed!" << std::endl;
        if(! (A_ch == A_ch_check) )
        {
            std::cout << "Old A:" << std::endl << A_ch.toString() << std::endl;
            std::cout << "New A:" << std::endl << A_ch_check.toString() << std::endl;
            std::cout << "Had originally " << points.size() -1 << " points in the Convex Hull, "
                      << " now we have " << points_check.size() << std::endl;
        }
        point_old = point;
    }

    _log << "];" << std::endl;

    if(footStrategy == USE_TASK)
    {
        _log << "points=[";
        for(KDL::Vector point : points)
            _log << point.x() << "," << point.y() << ";";
        _log << "];" << std::endl;
        _log << "points_inner=[";
        for(KDL::Vector point : points_inner)
            _log << point.x() << "," << point.y() << ";";
        _log << "];" << std::endl;

        _log << "figure; hold on; plot2(points,'r'); plot2(points_inner,'g'); axis equal;" << std::endl;
        _log << "ct = plot2(com_traj,'b');" << std::endl;
    }
    else if(footStrategy==USE_CONSTRAINT)
    {
        _log << "ctc = plot2(com_traj_constraint,'m');" << std::endl;
        _log << "legend([ct,ctc], 'CoM Traj, r_sole ctrl as task', 'CoM Traj, r_sole ctrl as constraint');" << std::endl;
    }
*/
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
