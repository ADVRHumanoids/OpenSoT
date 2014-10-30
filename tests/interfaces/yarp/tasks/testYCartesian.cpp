#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <drc_shared/tests_utils.h>

namespace {

class testYTask : public ::testing::Test{
 protected:

  testYTask():
      q(robot.coman_iDyn3.getNrOfDOFs(), 0.0)
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

TEST_F(testYTask, testConstructors)
{
    std::string robot_name = "coman";
    std::string module_prefix = "sot_velkincon";
    std::string task_id = "cartesian::left_arm";
    std::string base_link = "Waist";
    std::string distal_link = "l_wrist";

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
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm(robot_name, module_prefix, task_id, q, robot, distal_link, base_link);

            EXPECT_EQ(y_cartesian_l_arm.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm.getTaskID(), task_id);
            std::string port_prefix("/"+robot_name+"/"+module_prefix+"/"+task_id+"/");
            EXPECT_EQ(y_cartesian_l_arm.getPortPrefix(), port_prefix);

            EXPECT_TRUE(yarp_network.exists(port_prefix+"set_ref:i"));

            OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_l_arm(
                        new OpenSoT::tasks::velocity::Cartesian(task_id, q, robot, distal_link, base_link));
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm2(robot_name, module_prefix, cartesian_l_arm);
            EXPECT_EQ(y_cartesian_l_arm2.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm2.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm2.getTaskID(), task_id);
            EXPECT_EQ(y_cartesian_l_arm2.getPortPrefix(), port_prefix);

            robot_name = "";
            module_prefix = "sot_velkincon";
            task_id = "cartesian::left_arm";
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm3(robot_name, module_prefix, task_id, q, robot, distal_link, base_link);
            EXPECT_EQ(y_cartesian_l_arm3.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm3.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm3.getTaskID(), task_id);
            port_prefix = "/"+module_prefix+"/"+task_id+"/";
            EXPECT_EQ(y_cartesian_l_arm3.getPortPrefix(), port_prefix);

            robot_name = "";
            module_prefix = "";
            task_id = "cartesian::left_arm";
            OpenSoT::interfaces::yarp::tasks::YCartesian y_cartesian_l_arm4(robot_name, module_prefix, task_id, q, robot, distal_link, base_link);
            EXPECT_EQ(y_cartesian_l_arm4.getBaseLink(), base_link);
            EXPECT_EQ(y_cartesian_l_arm4.getDistalLink(), distal_link);
            EXPECT_EQ(y_cartesian_l_arm4.getTaskID(), task_id);
            port_prefix = "/"+task_id+"/";
            EXPECT_EQ(y_cartesian_l_arm4.getPortPrefix(), port_prefix);
        }
        tests_utils::stopYarpServer();

    }

}

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
