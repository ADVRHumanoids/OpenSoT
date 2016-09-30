#include <gtest/gtest.h>
#include <OpenSoT/tasks/virtual_model/CartesianSpringDamper.h>
#include <OpenSoT/solvers/QPOases.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/virtual_model/TorqueLimits.h>
#include <numeric>
#include <qpOASES.hpp>

#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RED "\033[0;31m"
#define DEFAULT "\033[0m"

using namespace OpenSoT::tasks::virtual_model;
using namespace yarp::math;

namespace{

class testCartesianSpringDamper : public ::testing::Test {
 protected:

  testCartesianSpringDamper()
  {

  }

  void init(const int dofs)
  {
      tau.resize(dofs); tau.zero();
      q.resize(dofs); q.zero();
      q_dot.resize(dofs); q_dot.zero();
  }

  virtual ~testCartesianSpringDamper() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


  yarp::sig::Vector tau;
  yarp::sig::Vector q;
  yarp::sig::Vector q_dot;
};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
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

#if OPENSOT_COMPILE_SIMULATION_TESTS
TEST_F(testCartesianSpringDamper, static_test) {
    tests_utils::startYarpServer();

    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.world";
    //if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    //else
    //    tests_utils::startGZServer(world_path);

    yarp::os::Time::delay(4);

    RobotUtils bigman("static_test",
                      "bigman",
                      std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                      std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");

    init(bigman.idynutils.iDyn3_model.getNrOfDOFs());

    q = getGoodInitialPosition(bigman.idynutils);

    bigman.setPositionMode();
    double speed = 0.8;
    bigman.left_arm.setReferenceSpeed(speed);
    bigman.right_arm.setReferenceSpeed(speed);
    bigman.torso.setReferenceSpeed(speed);
    bigman.move(q);

    yarp::os::Time::delay(4);

    yarp::sig::Vector tau_m;
    bigman.idynutils.updateiDyn3Model(q, q_dot, true);


    OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr boundsTorqueLimits =
            OpenSoT::constraints::virtual_model::TorqueLimits::ConstraintPtr(
                new OpenSoT::constraints::virtual_model::TorqueLimits(
                    bigman.idynutils.iDyn3_model.getJointTorqueMax()));

    OpenSoT::tasks::virtual_model::CartesianSpringDamper::Ptr spring_damper_task_r_wrist=
            OpenSoT::tasks::virtual_model::CartesianSpringDamper::Ptr(
                new OpenSoT::tasks::virtual_model::CartesianSpringDamper("spring_damper::r_wrist",
                    q, bigman.idynutils,"r_wrist", "RShp"));
    yarp::sig::Matrix K(6,6), D(6,6);
    spring_damper_task_r_wrist->getStiffnessDamping(K,D);
    K = 300.*K.eye();
    K(3,3) = 300.;
    K(4,4) = 300.;
    K(5,5) = 300.;
    D = 5.*D.eye();
    D(3,3) = 5.;
    D(4,4) = 5.;
    D(5,5) = 5.;
    spring_damper_task_r_wrist->setStiffnessDamping(K, D);

    yarp::sig::Matrix ref; yarp::sig::Vector ref_twist;
    spring_damper_task_r_wrist->getReference(ref, ref_twist);
    std::cout<<"reference: "<<std::endl;cartesian_utils::printHomogeneousTransform(ref);
    KDL::Frame actual_pose = bigman.idynutils.getPose("RShp","r_wrist");
    std::cout<<"actual_pose: "<<std::endl;cartesian_utils::printKDLFrame(actual_pose);

    std::cout<<"reference twist: "<<std::endl;cartesian_utils::printVelocityVector(ref_twist);

    yarp::sig::Vector spring_force = spring_damper_task_r_wrist->getSpringForce();
    std::cout<<"spring_force: ["<<spring_force.toString()<<"]"<<std::endl;

    yarp::sig::Vector damper_force = spring_damper_task_r_wrist->getDamperForce();
    std::cout<<"damper_force: ["<<damper_force.toString()<<"]"<<std::endl;

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(spring_damper_task_r_wrist);

    OpenSoT::solvers::QPOases_sot::Ptr sot;
    sot = OpenSoT::solvers::QPOases_sot::Ptr(
            new OpenSoT::solvers::QPOases_sot(stack_of_tasks, boundsTorqueLimits, 2e11));

    bool start_torque_ctrl = false;
    yarp::sig::Vector tau(q.size());
    while(1)
    {

        bigman.sense(q, q_dot, tau_m);
        bigman.idynutils.updateiDyn3Model(q, q_dot, true);

        boundsTorqueLimits->update(q);
        spring_damper_task_r_wrist->update(q);


        if(sot->solve(tau))
        {
            if(start_torque_ctrl)
            {
                yarp::sig::Vector tau_arm(bigman.right_arm.getNumberOfJoints());
                for(unsigned int i = 0; i < bigman.right_arm.getNumberOfJoints(); ++i)
                    tau_arm[i] = tau[bigman.idynutils.right_arm.joint_numbers[i]];

                bigman.right_arm.move(tau_arm);
            }
        }
        else
        {
            std::cout<<RED<<"SOLVER ERROR, EXITING"<<DEFAULT<<std::endl;
            break;
        }

        if(!start_torque_ctrl)
        {
            start_torque_ctrl = true;

            bool a = bigman.right_arm.setControlType(walkman::controlTypes::torque);
            if (a)
                std::cout<<GREEN<<"TORQUE CTRL STARTED"<<DEFAULT<<std::endl;
            else{
                std::cout<<RED<<"TORQUE CTRL CAN NOT START, EXITING"<<DEFAULT<<std::endl;
                break;
            }
        }

        if(tests_utils::_kbhit())
        {
            std::cout<<GREEN<<"USER PRESS A BUTTON, EXITINIG..."<<DEFAULT<<std::endl;
            break;
        }
    }


    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}
#endif
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
