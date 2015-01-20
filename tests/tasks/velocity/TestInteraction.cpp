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

yarp::sig::Vector computeWallForce(yarp::sig::Matrix& xw, yarp::sig::Matrix& xr, yarp::sig::Matrix& Kw)
{
    yarp::sig::Vector pe, oe;
    cartesian_utils::computeCartesianError(xr, xw, pe, oe);

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

    // To test the interaction task we assume a stiff model of the wall as:
    //          Ww = Kw*(xw - xr)
    // with Kw the stiffness of the wall, xw the position of the wall and xr the position of the robot.
    // We do not care about torques.
    yarp::sig::Matrix Kw(6,6); Kw = Kw.eye();
    Kw = 1E6 * Kw;
    yarp::sig::Matrix xw = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex("l_wrist"));
    yarp::sig::Vector Ww = computeWallForce(xw, xw, Kw);

    //Before setting the wrench, the wall wrench has to be transformed in the FT sensor frame!
    yarp::sig::Matrix ft_sensor_in_world = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex("l_arm_ft"));
    KDL::Frame ft_sensor_in_world_KDL;
    cartesian_utils::fromYARPMatrixtoKDLFrame(ft_sensor_in_world, ft_sensor_in_world_KDL);
    KDL::Wrench Ww_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(Ww, Ww_KDL);
    Ww_KDL = ft_sensor_in_world_KDL.Inverse()*Ww_KDL;
    cartesian_utils::fromKDLWrenchtoYarpVector(Ww_KDL, Ww);

    for(unsigned int i = 0; i < 4; ++i)
        _robot.iDyn3_model.setSensorMeasurement(i, -1.0*Ww);



    Interaction::Ptr interactionTask(new Interaction("interaction::l_wrist", q, _robot, "l_wrist", "world", "l_arm_ft"));

    yarp::sig::Matrix C = interactionTask->getCompliance();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) = 1E-6;
        C(i+3, i+3) = 1E-6;}


    interactionTask->setCompliance(C);

    EXPECT_TRUE(C == interactionTask->getCompliance());

//    std::vector<bool> active_joint_mask = interactionTask->getActiveJointsMask();
//    for(unsigned int i = 0; i < _robot.left_leg.getNrOfDOFs(); ++i)
//        active_joint_mask[_robot.left_leg.joint_numbers[i]] = false;
//    interactionTask->setActiveJointsMask(active_joint_mask);

    int t = 1000;
    VelocityLimits::Ptr joint_velocity_limits(new VelocityLimits(0.1, (double)(1.0/t), q.size()));

    JointLimits::Ptr joint_limits(new JointLimits(q, _robot.iDyn3_model.getJointBoundMax(),
                           _robot.iDyn3_model.getJointBoundMin()));

    //Create the SoT
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(interactionTask);

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    yarp::sig::Vector Wd = Ww;
    Wd[0] += 10.0;
    Wd[1] += 20.0;
    Wd[2] += 20.0;
    Wd[3] += 5.0;
    Wd[4] += 1.0;
    Wd[5] += 5.0;
    interactionTask->setReferenceWrench(Wd);

    yarp::sig::Vector dq(q.size(), 0.0);
//    for(unsigned int i = 0; i < 20*t; ++i)
    for(unsigned int i = 0; i < 2; ++i)
    {
        _robot.updateiDyn3Model(q, true);
        yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex("l_wrist"));
        std::cout<<"l_wrist in world: "<<std::endl;
        cartesian_utils::printHomogeneousTransform(x);
        Ww = computeWallForce(xw, x, Kw);
        std::cout<<"Wall Wrench in world: ["<<Ww.toString()<<"]"<<std::endl;

        //Before setting the wrench, the wall wrench has to be transformed in the FT sensor frame!
        yarp::sig::Matrix ft_sensor_in_world = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex("l_arm_ft"));
        KDL::Frame ft_sensor_in_world_KDL; cartesian_utils::fromYARPMatrixtoKDLFrame(ft_sensor_in_world, ft_sensor_in_world_KDL);
        KDL::Wrench Ww_KDL; cartesian_utils::fromYarpVectortoKDLWrench(Ww, Ww_KDL);
        Ww_KDL = ft_sensor_in_world_KDL.Inverse()*Ww_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(Ww_KDL, Ww);

         for(unsigned int i = 0; i < _robot.iDyn3_model.getNrOfFTSensors(); ++i)
             _robot.iDyn3_model.setSensorMeasurement(i, -1.0*Ww);

         interactionTask->update(q);
         joint_constraints->update(q);

         sot.solve(dq);
         q += dq;
    }

    _robot.updateiDyn3Model(q, true);
    yarp::sig::Vector expected = yarp::math::pinv(Kw) * Wd;
    yarp::sig::Vector pe, oe;
    yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(_robot.iDyn3_model.getLinkIndex("l_wrist"));
    cartesian_utils::computeCartesianError(x, xw, pe, oe);
    yarp::sig::Vector solution = yarp::math::cat(pe, oe);

    std::cout<<"EXPECTED:   "<<expected.toString()<<std::endl;
    std::cout<<"ACTUAL:     "<<solution.toString()<<std::endl;
}
}
