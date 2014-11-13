#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/interfaces/yarp/tasks/YPostural.h>
#include <idynutils/tests_utils.h>
#include <yarp/math/Math.h>

using namespace yarp::math;
using namespace OpenSoT::tasks::velocity;

namespace {

class testYTask : public ::testing::Test{
 protected:

  testYTask():
      q(robot.iDyn3_model.getNrOfDOFs(), 0.0)
  {
      robot.updateiDyn3Model(q, true);
  }

  virtual ~testYTask()
  {}

  virtual void SetUp()
  {}

  virtual void TearDown()
  {}

  iDynUtils robot;
  yarp::sig::Vector q;
};

TEST_F(testYTask, testYPostural)
{
    if(tests_utils::startYarpServer())
    {
        ::yarp::os::Network yarp_network;

        unsigned int number_of_trials = 1000;
        bool check = false;
        for(unsigned int i = 0; i < number_of_trials; ++i)
        {
            if(yarp_network.checkNetwork())
            {
                check = true;
                break;
            }
        }
        if(check)
        {
            std::string robot_name = "coman";
            std::string module_prefix = "sot_velkincon";

            Postural::Ptr postural_task(new Postural(q));
            OpenSoT::interfaces::yarp::tasks::YPostural y_postural_task(robot_name, module_prefix, robot, postural_task);


            yarp::os::BufferedPort<OpenSoT::interfaces::yarp::msgs::yarp_position_joint_msg_portable> joint_position_msg_port;
            joint_position_msg_port.open("/test_port:o");
            sleep(1);
            EXPECT_TRUE(yarp_network.connect("/test_port:o", y_postural_task.getPortPrefix()+"set_ref:i"));
            sleep(1);
            OpenSoT::interfaces::yarp::msgs::yarp_position_joint_msg_portable& references = joint_position_msg_port.prepare();
            for(unsigned int i = 0; i < robot.left_arm.joint_names.size(); ++i)
                references.joints[robot.left_arm.joint_names[i]] = M_PI/(i+1);

            std::cout<<"references size is: "<<references.bottle_msg_size()<<std::endl;
            EXPECT_EQ(references.bottle_msg_size(), 2*robot.left_arm.joint_names.size());

            joint_position_msg_port.write();
            sleep(1);

            std::cout<<"RECIVED POSTURE: ["<<y_postural_task.taskPostural->getReference().toString()<<" ]"<<std::endl;

            unsigned int j = 0;
            for(unsigned int i = 0; i < robot.iDyn3_model.getNrOfDOFs(); ++i)
            {
                if(std::find(robot.left_arm.joint_numbers.begin(),
                             robot.left_arm.joint_numbers.end(),
                             i) != robot.left_arm.joint_numbers.end()) {
                    EXPECT_DOUBLE_EQ(references.joints[robot.left_arm.joint_names[j]], y_postural_task.taskPostural->getReference()[i]);
                    j++;
                } else {
                    EXPECT_DOUBLE_EQ(0.0, y_postural_task.taskPostural->getReference()[i]);
                }
            }

            references = joint_position_msg_port.prepare();
            for(unsigned int i = 0; i < robot.right_arm.joint_names.size(); ++i)
                references.joints[robot.right_arm.joint_names[i]] = M_PI/(i+2);
            std::cout<<"references.joints size: "<<references.joints.size()<<std::endl;
            for(unsigned int i = 0; i < robot.right_leg.joint_names.size(); ++i)
                references.joints[robot.right_leg.joint_names[i]] = M_PI/(i+3);
            std::cout<<"references.joints size: "<<references.joints.size()<<std::endl;

            std::cout<<"references size is: "<<references.bottle_msg_size()<<std::endl;
            EXPECT_EQ(references.bottle_msg_size(), 2*(robot.right_arm.joint_names.size()+
                                                                  robot.right_leg.joint_names.size())+
                      2*robot.left_arm.joint_names.size());

            joint_position_msg_port.write();
            sleep(1);

            std::cout<<"RECIVED POSTURE: ["<<y_postural_task.taskPostural->getReference().toString()<<" ]"<<std::endl;

            unsigned int left_arm_index = 0;
            unsigned int right_arm_index = 0;
            unsigned int right_leg_index = 0;
            for(unsigned int i = 0; i < robot.iDyn3_model.getNrOfDOFs(); ++i)
            {
                if(std::find(robot.left_arm.joint_numbers.begin(),
                             robot.left_arm.joint_numbers.end(),
                             i) != robot.left_arm.joint_numbers.end()) {
                    EXPECT_DOUBLE_EQ(references.joints[robot.left_arm.joint_names[left_arm_index]], y_postural_task.taskPostural->getReference()[i]);
                    left_arm_index++;
                }
                else if(std::find(robot.right_arm.joint_numbers.begin(),
                             robot.right_arm.joint_numbers.end(),
                             i) != robot.right_arm.joint_numbers.end()) {
                    EXPECT_DOUBLE_EQ(references.joints[robot.right_arm.joint_names[right_arm_index]], y_postural_task.taskPostural->getReference()[i]);
                    right_arm_index++;
                }
                else if(std::find(robot.right_leg.joint_numbers.begin(),
                             robot.right_leg.joint_numbers.end(),
                             i) != robot.right_leg.joint_numbers.end()) {
                    EXPECT_DOUBLE_EQ(references.joints[robot.right_leg.joint_names[right_leg_index]], y_postural_task.taskPostural->getReference()[i]);
                    right_leg_index++;
                }
                else {
                    EXPECT_DOUBLE_EQ(0.0, y_postural_task.taskPostural->getReference()[i]);
                }
            }


        }
        tests_utils::stopYarpServer();
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
