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

typedef std::pair<KDL::Path::IdentifierType,bool> testType;

class testQPOases_TestFF: public ::testing::Test
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

    testQPOases_TestFF() :
        rotationInterpolationMethod(
            new KDL::RotationalInterpolation_SingleAxis())
    {
    }

    virtual ~testQPOases_TestFF() {
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

class testQPOases_CartesianFF: public testQPOases_TestFF,
        public ::testing::WithParamInterface<testType>
{
protected:

    testQPOases_CartesianFF()
    {
        _log.open("testQPOases_CartesianFF.m");
        _log << "testQPOases_FF_Cartesian_5cmfw_noerr" << std::endl;
        _log << "testQPOases_FF_Cartesian_5cmfw_1cmerr" << std::endl;
    }

    virtual ~testQPOases_CartesianFF() {
        _log.close();
    }


    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testQPOases_CoMAndPosturalFF: public testQPOases_TestFF,
        public ::testing::WithParamInterface<bool>
{
protected:

    testQPOases_CoMAndPosturalFF()
    {
        _log.open("testQPOases_CoMAndPosturalFF.m");
        _log << "testQPOases_FF_CoM_5cmfw_noerr" << std::endl;
        _log << "testQPOases_FF_CoM_5cmfw_1cmerr" << std::endl;
        _log << "testQPOases_FF_Postural_1drad_noerr" << std::endl;
        _log << "testQPOases_FF_Postural_1drad_1craderr" << std::endl;
    }

    virtual ~testQPOases_CoMAndPosturalFF() {
        _log.close();
    }


    virtual void SetUp() {

    }

    virtual void TearDown() {

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


TEST_P(testQPOases_CartesianFF, testCartesianFF)
{

    KDL::Path::IdentifierType  trajType = GetParam().first;
    bool hasInitialError = GetParam().second;

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

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(l_arm_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds,1e9));



    yarp::sig::Vector dq;

    double dt=3e-3;

    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist twist_estimate, previous_twist_estimate, desired_twist;
    yarp::sig::Matrix current_pose_y;


    dq = yarp::sig::Vector(q.size(), 0.0);
    q = getGoodInitialPosition(model);
    model.updateiDyn3Model(q, true);
    l_arm_task->update(q);
    postural_task->update(q);
    bounds->update(q);


    if(!hasInitialError) {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM CURRENT POSITION
         * meaning zero error at trajectory begin
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_Cartesian_5cmfw_noerr.m");

        current_pose_y = l_arm_task->getActualPose();
        YarptoKDL(current_pose_y,current_pose);
        get5cmFwdLinearTraj(current_pose);
        desired_pose = trajectory->Pos(0.0);

        l_arm_task->setLambda(.6);
        l_arm_task->setOrientationErrorGain(.1);

    } else {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM PERTURBED POSITION
         * 1cm error at trajectory startup
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_Cartesian_5cmfw_1cmerr.m");

        current_pose_y = l_arm_task->getActualPose();
        YarptoKDL(current_pose_y,current_pose);
        desired_pose = current_pose;
        desired_pose.p[0] = current_pose.p[0] + .01;
        get5cmFwdLinearTraj(desired_pose);
        desired_pose = trajectory->Pos(0.0);

        /* setting lambda lower than this can cause tracking problems
         * along the trajectory on secondary variables (e.g. the one that
         * we want fixed at 0) */
        l_arm_task->setLambda(.1);
        l_arm_task->setOrientationErrorGain(.1);
    }

    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    double t_loop = dt;
    double t_compute = 0;

    double previous_norm = -1;
    double current_norm;
    double previous_error = desired_pose.p[0] - current_pose.p[0];

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(333Hz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;

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

        // initializing previous norm
        if(previous_norm < 0)
            previous_norm = norm(l_arm_task->getb());

        // checking variation of gain during trajectory following
        if(t>=6)
            l_arm_task->setLambda(.6);

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

        /* first order fading filter -> to implement in Matlab
        double beta = 0.3; double G = 1-beta;
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_estimate[0] + G*(twist_measure - twist_estimate[0]);
        */
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_measure;

        current_norm = norm(l_arm_task->getb());

        _log << t << ",\t"
             << twist_estimate[0] << ",\t"
             << desired_twist[0]   << ",\t"
             << current_pose.p[0]  << ",\t"
             << desired_pose.p[0]  << ",\t"
             << t_compute          << ",\t"
             << t_loop             << ",\t"
             << current_norm       << ";" << std::endl;
        // also velocities and accelerations are available !
        previous_pose = current_pose;
        previous_twist_estimate[0] = twist_estimate[0];

        if(!hasInitialError) {
            EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4);
            EXPECT_NEAR(norm(l_arm_task->getb()), 0, 5e-4);
        } else {
            if(t<=1.3) {
                double current_error = desired_pose.p[0] - current_pose.p[0];

                /* error should always decrease, or at least accept
                 * a local increment of 1e-4 */
                EXPECT_GE(previous_error - current_error, -1e-4) << " @t= " << t;
                EXPECT_GE(previous_norm - current_norm, -8e-4) << " @t= " << t;

                previous_error = current_error;
                previous_norm = current_norm;
            } else {

                EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4) << " @t= " << t;
                EXPECT_NEAR(norm(l_arm_task->getb()), 0, 8e-4) << " @t= " << t;
            }
        }
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "%moving average filter" << std::endl;
    _log << "filt_window = 25;" << std::endl;
    _log << "a = 1; b = 1/filt_window*ones(1,filt_window);" << std::endl;
    _log << "twist_estimate = filter(b,a,pos_des_x(:,2));" << std::endl;
    _log << "plot(pos_des_x(:,1),[twist_estimate, pos_des_x(:,3)]);" << std::endl;
    _log << "legend('Desired Velocity Profile', 'Actual Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Desired Position', 'Actual Position','Location','SouthEast');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,6:7)); title('Computation time'); legend('Solve time','loop time (333Hz)');" << std::endl;

}







TEST_P(testQPOases_CoMAndPosturalFF, testCoMFF)
{
    bool hasInitialError = GetParam();

#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("testCoMFF");
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

    OpenSoT::tasks::velocity::CoM::Ptr com(
                new OpenSoT::tasks::velocity::CoM(q, model));

    // Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;

    stack_of_tasks.push_back(com);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack_of_tasks, bounds,1e9));



    yarp::sig::Vector dq;

    double dt=3e-3;

    KDL::Frame current_pose, previous_pose, desired_pose;
    KDL::Twist twist_estimate, previous_twist_estimate, desired_twist;
    yarp::sig::Vector current_position_y;


    dq = yarp::sig::Vector(q.size(), 0.0);
    q = getGoodInitialPosition(model);
    model.updateiDyn3Model(q, true);
    com->update(q);
    postural_task->update(q);
    bounds->update(q);


    if(!hasInitialError) {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM CURRENT POSITION
         * meaning zero error at trajectory begin
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_Postural_1drad_noerr.m");

        current_position_y = com->getActualPosition();
        YarptoKDL(current_position_y,current_pose.p);
        get5cmFwdLinearTraj(current_pose);
        desired_pose = trajectory->Pos(0.0);

        com->setLambda(.6);

    } else {
        /**********************************************
         * COMMANDING 5cm FORWARD FROM PERTURBED POSITION
         * 1cm error at trajectory startup
         ********************************************/

        _log.close();
        _log.open("testQPOases_FF_Postural_1drad_1craderr.m");

        current_pose_y = com->getActualPose();
        YarptoKDL(current_position_y,current_pose);
        desired_pose = current_pose;
        desired_pose.p[0] = current_pose.p[0] + .01;
        get5cmFwdLinearTraj(desired_pose);
        desired_pose = trajectory->Pos(0.0);

        /* setting lambda lower than this can cause tracking problems
         * along the trajectory on secondary variables (e.g. the one that
         * we want fixed at 0) */
        com->setLambda(.1);
    }

    previous_pose = current_pose;
    desired_twist = trajectory->Vel(0.0);

    double t_loop = dt;
    double t_compute = 0;

    double previous_norm = -1;
    double current_norm;
    double previous_error = desired_pose.p[0] - current_pose.p[0];

    _log << "% t,\t"
         << "estimated_twist,\t"
         << "desired_twist,\t"
         << "current_pose,\t"
         << "desired_pose,\t"
         << "t_update_and_solve,\t"
         << "t_loop(333Hz)" << std::endl;
    _log << "pos_des_x = [" << std::endl;

    for (double t=0.0; t <= trajectory->Duration(); t+= t_loop)
    {
        double t_begin = yarp::os::SystemClock::nowSystem();
        yarp::sig::Vector desired_position_y(4,0.0);
        yarp::sig::Vector desired_twist_y(6,0.0);
        desired_pose = trajectory->Pos(t);
        desired_twist = trajectory->Vel(t);
        KDLtoYarp(desired_pose.p, desired_position_y);
        KDLtoYarp(desired_twist, desired_twist_y);
        com->setReference(desired_position_y, desired_twist_y*t_loop);

        // initializing previous norm
        if(previous_norm < 0)
            previous_norm = norm(com->getb());

        // checking variation of gain during trajectory following
        if(t>=6)
            com->setLambda(.6);

        model.updateiDyn3Model(q, true);

        com->update(q);
        postural_task->update(q);
        bounds->update(q);

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

        current_position_y = com->getActualPosition();
        YarptoKDL(current_position_y,current_pose.p);


        t_compute = yarp::os::SystemClock::nowSystem() - t_begin;
        yarp::os::SystemClock::delaySystem(dt-t_compute);
        t_loop = yarp::os::SystemClock::nowSystem() - t_begin;

        /* first order fading filter -> to implement in Matlab
        double beta = 0.3; double G = 1-beta;
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_estimate[0] + G*(twist_measure - twist_estimate[0]);
        */
        double twist_measure = (current_pose.p[0] - previous_pose.p[0])/t_loop;
        twist_estimate[0] = twist_measure;

        current_norm = norm(com->getb());

        _log << t << ",\t"
             << twist_estimate[0] << ",\t"
             << desired_twist[0]   << ",\t"
             << current_pose.p[0]  << ",\t"
             << desired_pose.p[0]  << ",\t"
             << t_compute          << ",\t"
             << t_loop             << ",\t"
             << current_norm       << ";" << std::endl;
        // also velocities and accelerations are available !
        previous_pose = current_pose;
        previous_twist_estimate[0] = twist_estimate[0];

        if(!hasInitialError) {
            EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4);
            EXPECT_NEAR(current_norm, 0, 5e-4);
        } else {
            if(t<=1.3) {
                double current_error = desired_pose.p[0] - current_pose.p[0];

                /* error should always decrease, or at least accept
                 * a local increment of 1e-4 */
                EXPECT_GE(previous_error - current_error, -1e-4) << " @t= " << t;
                EXPECT_GE(previous_norm - current_norm, -8e-4) << " @t= " << t;

                previous_error = current_error;
                previous_norm = current_norm;
            } else {

                EXPECT_NEAR(current_pose.p[0], desired_pose.p[0],1e-4) << " @t= " << t;
                EXPECT_NEAR(current_norm, 0, 8e-4) << " @t= " << t;
            }
        }
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "subplot(2,1,1);" << std::endl;
    _log << "%moving average filter" << std::endl;
    _log << "filt_window = 25;" << std::endl;
    _log << "a = 1; b = 1/filt_window*ones(1,filt_window);" << std::endl;
    _log << "twist_estimate = filter(b,a,pos_des_x(:,2));" << std::endl;
    _log << "plot(pos_des_x(:,1),[twist_estimate, pos_des_x(:,3)]);" << std::endl;
    _log << "legend('Desired Velocity Profile', 'Actual Velocity Profile');" << std::endl;
    _log << "subplot(2,1,2);" << std::endl;
    _log << "plot(pos_des_x(:,1),pos_des_x(:,4:5));" << std::endl;
    _log << "legend('Desired Position', 'Actual Position','Location','SouthEast');" << std::endl;
    _log << "figure; plot(pos_des_x(:,1),pos_des_x(:,6:7)); title('Computation time'); legend('Solve time','loop time (333Hz)');" << std::endl;

}

TEST_P(testQPOases_CoMAndPosturalFF, testPosturalFF)
{

}

INSTANTIATE_TEST_CASE_P(FFTests,
                        testQPOases_CartesianFF,
                        ::testing::Values(
    std::make_pair(KDL::Path::ID_LINE,false),
    std::make_pair(KDL::Path::ID_LINE, true),
    std::make_pair(KDL::Path::ID_CIRCLE, true),
    std::make_pair(KDL::Path::ID_CIRCLE, false)));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
