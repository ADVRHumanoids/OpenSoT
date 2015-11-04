#include <gtest/gtest.h>
#include <utils.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <yarp/sig/Vector.h>
#include <string>

using namespace yarp::math;

namespace {

// The fixture for testing class Foo.
class testKlamptController : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  testKlamptController() {
    // You can do set-up work for each test here.
  }

  virtual ~testKlamptController() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(testKlamptController, mapsWork) {
    iDynUtils robot("huboplus",
        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"huboplus/huboplus.urdf",
        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"huboplus/huboplus.srdf");

    yarp::sig::Vector q, q_check;
    KlamptController::JntPosition pose;
    q = tests_utils::getRandomAngles(yarp::sig::Vector(robot.getJointNames().size(), 0.0),
                                     yarp::sig::Vector(robot.getJointNames().size(), 6.28),
                                     robot.getJointNames().size());
    pose = fromiDynToJnt(robot, q);
    q_check = fromJntToiDyn(robot, pose);
    for(unsigned int i = 0; i < robot.getJointNames().size(); ++i)
    {
        EXPECT_DOUBLE_EQ(q[i],q_check[i]) << "Error converting @joint" << robot.getJointNames()[i]
                                          << " to/from joint map" << std::endl;
    }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
