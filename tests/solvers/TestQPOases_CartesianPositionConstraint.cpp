#include <iCub/iDynTree/yarp_kdl.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
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
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>


using namespace yarp::math;

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

namespace {

class CircularTrajectory {
protected:
    KDL::Frame      _start;
    KDL::Frame      _center;
    double          _radius;
    KDL::Vector     _rotationAxis;
    double          _speed; //[m/s]
public:
    CircularTrajectory(const KDL::Frame &start,
                       const KDL::Frame &center,
                       const KDL::Vector &rotationAxis,
                       const double speed) {
      this->_start = start;
      this->_center = center;
      this->_rotationAxis = rotationAxis;
      this->_radius = (start.p - center.p).Norm();
      this->_speed = speed;
      if(KDL::dot(_rotationAxis,(center.p-start.p)) != 0) {
          std::cout << "Error: dot product of rotation axis and radius vector is "
                    << KDL::dot(_rotationAxis,(center.p-start.p)) << ", should have been 0" << std::endl;
          throw(new std::runtime_error("rotation axis and radius vector are not normal!"));
      }
    }

    // get a constant speed circular trajectory
    KDL::Frame Pos(const double time) {
        KDL::Frame translation;
        translation.p = KDL::Rotation::Rot( _rotationAxis,
                                            2*M_PI*time/this->Duration())*(_start.p - _center.p);
        KDL::Frame actual = translation*this->_center;
        return actual;
    }

    double pathLength() {
        return this->_radius*2.0*M_PI;
    }

    double Duration() {
        return this->pathLength()/this->_speed;
    }

};

class testQPOases_CartesianPositionConstraint:
        public ::testing::Test
{
protected:
    std::ofstream   _log;

    testQPOases_CartesianPositionConstraint()
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
};


yarp::sig::Vector getGoodInitialPosition(iDynUtils& model) {
    yarp::sig::Vector q(model.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(model.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    model.fromRobotToIDyn(leg, q, model.left_leg);
    model.fromRobotToIDyn(leg, q, model.right_leg);
    yarp::sig::Vector arm(model.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    model.fromRobotToIDyn(arm, q, model.left_arm);
    arm[1] = -arm[1];
    model.fromRobotToIDyn(arm, q, model.right_arm);
    return q;
}

//#define TRY_ON_SIMULATOR
TEST_F(testQPOases_CartesianPositionConstraint, tryFollowingBounds) {

#ifdef TRY_ON_SIMULATOR
    yarp::os::Network init;
    ComanUtils robot("hitTheBounds");
#endif

    iDynUtils model("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(model);
    model.switchAnchorAndFloatingBase(model.left_leg.end_effector_name);
    model.updateiDyn3Model(q, true);

#ifdef TRY_ON_SIMULATOR
    robot.setPositionDirectMode();
    robot.move(q);
    yarp::os::Time::delay(3);
#endif

    OpenSoT::AutoStack::Ptr stack;
    OpenSoT::DefaultHumanoidStack DHS(model, 3e-3, q);

    yarp::sig::Matrix A_Cartesian(2,3); yarp::sig::Vector b_Cartesian(2,0.0);
    // setting constraints so that the end-effector z-coordinate
    // is bounded to be between current_z+0.4 and current_z-0.4
    A_Cartesian(0,2) = 1; A_Cartesian(1,2) = -1;
    b_Cartesian(0) = DHS.leftArm->getActualPose()(2,3)+0.04;
    b_Cartesian(1) = -(DHS.leftArm->getActualPose()(2,3)-0.04);

    std::cout << "A: \n" << A_Cartesian.toString() << std::endl;
    std::cout << "b: \n" << b_Cartesian.toString() << std::endl;

    namespace vConstraints = OpenSoT::constraints::velocity;
    vConstraints::CartesianPositionConstraint::Ptr cartesianConstraint(
        new vConstraints::CartesianPositionConstraint(q,
                                                      DHS.leftArm,
                                                      A_Cartesian,
                                                      b_Cartesian, 0.5));
    DHS.velocityLimits->setVelocityLimits(0.3);
    DHS.com->setLambda(0.6);
    yarp::sig::Matrix W(3,3);
    W.eye(); W(2,2) = .1;
    DHS.com->setWeight(W);
    W.resize(6,6);
    W.eye(); W(0,0) = .5;
    DHS.leftArm->setLambda(0.3);
    DHS.leftArm->setOrientationErrorGain(0.05);

    stack =  (DHS.leftLeg + DHS.rightLeg) /
             ((DHS.leftArm << cartesianConstraint) + DHS.rightArm) /
             //(DHS.leftArm + DHS.rightArm) /
             DHS.com / DHS.postural;
    stack << DHS.jointLimits << DHS.velocityLimits;

    OpenSoT::solvers::QPOases_sot::Ptr sot(
        new OpenSoT::solvers::QPOases_sot(stack->getStack(),
                                          stack->getBounds()));


    //SET SOME REFERENCES

    KDL::Frame start, center;
    yarp::sig::Matrix start_y = DHS.leftArm->getActualPose();
    YarptoKDL(start_y,start);
    center = start;

    double radius = 0.05;
    center.p.y(start.p.y()-radius);

    // rotating along the x axis
    CircularTrajectory traj1(start, center, KDL::Vector(1.0,0.0,0.0), 0.05);
    ASSERT_TRUE(traj1.Pos(0.0) == start) << start << "\nis different than\n"
                                         << traj1.Pos(0.0);


    yarp::sig::Vector dq(q.size(), 0.0);
    double e;

    unsigned int i = 0;
    _log << "start = [" << start.p.y() << "," << start.p.z() << "];" << std::endl;
    _log << "center = [" << center.p.y() << "," << center.p.z() << "];" << std::endl;
    _log << "%t, des_y, des_z, act_y, act_z, expected_y, expected_z, e, t_loop" << std::endl;
    _log << "test_data = [";

    double t_loop = 0.0;
    double t_test = yarp::os::SystemClock::nowSystem();
    for (double t=0.0; t <= traj1.Duration(); t+= t_loop)
    {
        double t_begin = yarp::os::SystemClock::nowSystem();

        model.updateiDyn3Model(q, true);
        stack->update(q);

        // setting left arm reference
        KDL::Frame traj_pose = traj1.Pos(t);
        yarp::sig::Matrix traj_pose_y;
        KDLtoYarp_position(traj_pose, traj_pose_y);
        DHS.leftArm->setReference(traj_pose_y);

        // setting postural reference to have minimum velocity
        // on left arm, postural on the rest of the body
        yarp::sig::Vector q_postural = DHS.postural->getReference();
        for(unsigned int i = 0; i < model.left_arm.joint_numbers.size(); ++i)
            q_postural[model.left_arm.joint_numbers[i]] = q[model.left_arm.joint_numbers[i]];
        DHS.postural->setReference(q_postural);

        // we get only the position, the orientation remains fixed
        KDL::Frame desired_pose = start;
        desired_pose.p = traj_pose.p;
        KDL::Frame expected_pose = desired_pose;
        KDL::Frame actual_pose;
        yarp::sig::Matrix actual_pose_y = DHS.leftArm->getActualPose();
        YarptoKDL(actual_pose_y, actual_pose);

        if(desired_pose.p.z() > b_Cartesian(0))
            expected_pose.p.z(b_Cartesian(0));
        if(desired_pose.p.z() < -b_Cartesian(1))
            expected_pose.p.z(-b_Cartesian(1));

        e = norm(DHS.leftArm->getb());

        EXPECT_TRUE(sot->solve(dq));
        q += dq;

#ifdef TRY_ON_SIMULATOR
            robot.move(q);
            yarp::os::Time::delay(0.05);
#endif

        t_loop = yarp::os::SystemClock::nowSystem() - t_begin;

        _log << yarp::os::SystemClock::nowSystem() - t_test << ","
             << desired_pose.p.y() << "," << desired_pose.p.z() << ","
             << actual_pose.p.y() << "," << actual_pose.p.z() << ","
             << expected_pose.p.y() << "," << expected_pose.p.z() << ","
             << e << "," << t_loop << ";" << std::endl;
    }

    _log << "];" << std::endl;

    _log << "figure" << std::endl;
    _log << "plot(test_data(:,2),test_data(:,3),'r');" << std::endl;
    _log << "hold on;" << std::endl;
    _log << "plot(test_data(:,4),test_data(:,5),'g');" << std::endl;
    _log << "plot(test_data(:,6),test_data(:,7),'b');" << std::endl;
    _log << "plot(center(1), center(2), 'co');"<< std::endl;
    _log << "plot(start(1),  start(2),  'm+');"<< std::endl;
    _log << "axis square;" << std::endl;
    _log << "title('End Effector Trajectories on the YZ plane');" << std::endl;
    _log << "legend('Desired','Actual','Expected');" << std::endl;
    _log << "figure; plot(test_data(:,1),test_data(:,9)); title('Computation time'); legend('Solve time');" << std::endl;
    _log << "figure; plot(test_data(:,1),test_data(:,8)); title('Tracking Error'); legend('Left Arm Trajectory tracking error');" << std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
