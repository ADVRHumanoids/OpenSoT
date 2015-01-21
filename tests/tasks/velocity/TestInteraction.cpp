#include <idynutils/tests_utils.h>
#include <idynutils/cartesian_utils.h>
#include <idynutils/idynutils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>

using namespace yarp::math;
using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

namespace {

class testInteractionTask: public ::testing::Test
{
protected:
    testInteractionTask()
    {

    }

    virtual ~testInteractionTask() {

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

yarp::sig::Vector computeWallForce(yarp::sig::Matrix& xd, yarp::sig::Matrix& x, const yarp::sig::Matrix& Kw)
{
    yarp::sig::Vector pe, oe;
    cartesian_utils::computeCartesianError(x, xd, pe, oe);

    return Kw * yarp::math::cat(pe, oe);
}

TEST_F(testInteractionTask, testInteractionTask_wrench)
{
    std::string urdf_file = std::string(getenv("WALKMAN_ROOT")) + "/drc/OpenSoT/tests/tasks/velocity/bigman.urdf";
    std::string srdf_file = std::string(getenv("WALKMAN_ROOT")) + "/drc/OpenSoT/tests/tasks/velocity/bigman.srdf";

    iDynUtils _robot("bigman", urdf_file, srdf_file);

    yarp::sig::Vector q = getGoodInitialPosition(_robot);
    _robot.updateiDyn3Model(q, true);

    _robot.setFloatingBaseLink(_robot.left_leg.end_effector_name);

    std::string distal_link = "l_wrist";
    std::string ft_sensor_link = "l_arm_ft";
    std::string base_link = "torso";

    // We consider the robot sturting already in contact:
    yarp::sig::Matrix xw = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(ft_sensor_link));
    // The robot has a certain Compliance:
    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) = 1E-4;
        C(i+3, i+3) = 1E-1;}
    // We compute the contact force as:
    yarp::sig::Vector base_link_Ww = computeWallForce(xw, xw, yarp::math::pinv(C));
    // We have to report the wrench in sensor frame
    KDL::Frame xw_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(xw, xw_KDL);
    KDL::Wrench base_link_Ww_KDL; cartesian_utils::fromYarpVectortoKDLWrench(base_link_Ww, base_link_Ww_KDL);
    KDL::Wrench sensor_Ww_KDL = xw_KDL.Inverse()*base_link_Ww_KDL;
    yarp::sig::Vector sensor_Ww; cartesian_utils::fromKDLWrenchtoYarpVector(sensor_Ww_KDL, sensor_Ww);

    // And we set as a feedback
    for(unsigned int i = 0; i < 4; ++i)
        _robot.iDyn3_model.setSensorMeasurement(i, -1.0*sensor_Ww);

    Interaction::Ptr interactionTask(new Interaction("interaction::l_wrist", q, _robot, distal_link,
                                                     base_link, ft_sensor_link));

    interactionTask->setCompliance(C);

    EXPECT_TRUE(C == interactionTask->getCompliance());

    std::vector<bool> active_joint_mask = interactionTask->getActiveJointsMask();
    for(unsigned int i = 0; i < _robot.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[_robot.left_leg.joint_numbers[i]] = false;
    interactionTask->setActiveJointsMask(active_joint_mask);

    int t = 1000;
    VelocityLimits::Ptr joint_velocity_limits(new VelocityLimits(0.1, (double)(1.0/t), q.size()));

    JointLimits::Ptr joint_limits(new JointLimits(q, _robot.iDyn3_model.getJointBoundMax(),
                           _robot.iDyn3_model.getJointBoundMin()));

    //Create the SoT
    std::vector< OpenSoT::Task<Matrix, Vector>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(interactionTask);

    std::list< OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    yarp::sig::Vector base_link_Wd = interactionTask->getActualWrench();
    base_link_Wd[0] += 20.0;
    base_link_Wd[1] += 10.0;
    base_link_Wd[2] += 10.0;
    base_link_Wd[3] += 5.0;
    base_link_Wd[4] += 1.0;
    base_link_Wd[5] += 5.0;
    interactionTask->setReferenceWrench(base_link_Wd);

    yarp::sig::Vector dq(q.size(), 0.0);
    //for(unsigned int i = 0; i < 20*t; ++i)
    for(unsigned int i = 0; i < 20; ++i)
    {
        _robot.updateiDyn3Model(q, false);
        yarp::sig::Matrix xd = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(ft_sensor_link));
        std::cout<<ft_sensor_link<<" desired in world: "<<std::endl;
        cartesian_utils::printHomogeneousTransform(xd);
        base_link_Ww = computeWallForce(xd, xw, pinv(C));
        // We have to report the wrench in sensor frame
        cartesian_utils::fromYARPMatrixtoKDLFrame(xw, xw_KDL);
        cartesian_utils::fromYarpVectortoKDLWrench(base_link_Ww, base_link_Ww_KDL);
        sensor_Ww_KDL = xw_KDL.Inverse()*base_link_Ww_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(sensor_Ww_KDL, sensor_Ww);
        std::cout<<"Wall Wrench in sensor frame: ["<<sensor_Ww.toString()<<"]"<<std::endl;

         for(unsigned int i = 0; i < _robot.iDyn3_model.getNrOfFTSensors(); ++i)
             _robot.iDyn3_model.setSensorMeasurement(i, -1.0*sensor_Ww);

         interactionTask->update(q);
         joint_constraints->update(q);

         std::cout<<"Force error: ["<<interactionTask->forceError.toString()<<"]"<<std::endl;
         std::cout<<"Torque error: ["<<interactionTask->torqueError.toString()<<"]"<<std::endl;

         sot.solve(dq);
         q += dq;

         std::cout<<std::endl;
    }

    _robot.updateiDyn3Model(q, true);
    yarp::sig::Vector expected = C * base_link_Wd;
    yarp::sig::Vector pe, oe;
    yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex(ft_sensor_link));
    cartesian_utils::computeCartesianError(x, xw, pe, oe);
    yarp::sig::Vector solution = yarp::math::cat(pe, oe);

    std::cout<<"EXPECTED:   "<<expected.toString()<<std::endl;
    std::cout<<"ACTUAL:     "<<solution.toString()<<std::endl;
}
}
