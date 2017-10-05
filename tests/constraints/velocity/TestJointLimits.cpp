#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <XBotInterface/ModelInterface.h>
#include <cmath>
#define  s 1.0

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace {

// The fixture for testing class JointLimits.
class testJointLimits : public ::testing::Test {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testJointLimits()
  {

      _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      _model_ptr->getJointLimits(qLowerBounds, qUpperBounds);
      zeros.setZero(_model_ptr->getJointNum());

      jointLimits = new OpenSoT::constraints::velocity::JointLimits(zeros,
                                    qUpperBounds, qLowerBounds);
  }

  virtual ~testJointLimits() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(jointLimits != NULL) {
        delete jointLimits;
        jointLimits = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _model_ptr->setJointPosition(zeros);
      _model_ptr->update();
      jointLimits->update(zeros);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for JointLimits.

  XBot::ModelInterface::Ptr _model_ptr;
  OpenSoT::constraints::velocity::JointLimits* jointLimits;

  Eigen::VectorXd qLowerBounds;
  Eigen::VectorXd qUpperBounds;
  Eigen::VectorXd zeros;
  Eigen::VectorXd q;
};

TEST_F(testJointLimits, sizesAreCorrect) {
    unsigned int x_size = _model_ptr->getJointNum();

    Eigen::VectorXd lowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd upperBound = jointLimits->getUpperBound();

    EXPECT_EQ(x_size, lowerBound.size()) << "lowerBound should have size"
                                         << x_size;
    EXPECT_EQ(x_size, upperBound.size()) << "upperBound should have size"
                                         << x_size;

    EXPECT_EQ(0, jointLimits->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << jointLimits->getAeq().rows();

    EXPECT_EQ(0, jointLimits->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               << jointLimits->getbeq().size();

    EXPECT_EQ(0,jointLimits->getAineq().rows()) << "Aineq should have size 0"
                                                << "but has size"
                                                << jointLimits->getAeq().rows();

    EXPECT_EQ(0,jointLimits->getbLowerBound().size()) << "beq should have size 0"
                                                      << "but has size"
                                                      << jointLimits->getbLowerBound().size();

    EXPECT_EQ(0,jointLimits->getbUpperBound().size()) << "beq should have size 0"
                                                      << "but has size"
                                                      << jointLimits->getbUpperBound().size();
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testJointLimits, BoundsAreCorrect) {

    Eigen::VectorXd q = zeros;
    q[16] = qLowerBounds[16] - 1E-1;
    q[17] = qLowerBounds[17];
    q[19] = qUpperBounds[19];
    q[20] = qUpperBounds[20] + 1E-1;

    q[22] = (qUpperBounds[22] + qLowerBounds[22])/2;
    q[23] = (qUpperBounds[22] + qLowerBounds[22])/2 - 1E-1;
    q[24] = (qUpperBounds[22] + qLowerBounds[22])/2 + 1E-1;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    jointLimits->update(q);
    Eigen::VectorXd lowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd upperBound = jointLimits->getUpperBound();

    /* checking a joint outside bounds
    EXPECT_DOUBLE_EQ(0.0, lowerBound[16]) << "Joint 16 below lower bound " << q[16] << std::endl
                                 << "Lower Velocity Limits should be 0,"
                                 << lowerBound[16] << "instead";
    EXPECT_GT(0.0, upperBound[16]) << "Joint 16 at lower bound " << q[16] << std::endl
                                   << "Lower Velocity Limits should be > 0,"
                                   << upperBound[16] << "instead";
    */

    /* checking a joint at upper bound */
    EXPECT_DOUBLE_EQ(0, lowerBound[17]) << "Joint 17 at lower bound " << q[17] << std::endl
                                        << "Lower Velocity Limits should be 0, "
                                        << lowerBound[17] << " instead";
    EXPECT_GT(upperBound[17], 0.0) << "Joint 17 at lower bound " << q[17] << std::endl
                                   << "Upper Velocity Limits should be > 0, "
                                   << upperBound[17] << " instead";

    /* checking a joint at lower bound */
    EXPECT_DOUBLE_EQ(0.0, upperBound[19]) << "Joint 19 at upper bound " << q[19] << std::endl
                                          << "Upper Velocity Limits should be 0, "
                                          << upperBound[19] << " instead";
    EXPECT_LT(lowerBound[19], 0.0) << "Joint 19 at upper bound " << q[19] << std::endl
                                   << "Lower Velocity Limits should be < 0 "
                                   << lowerBound[19] << " instead";

    /* checking a joint above upper bound
    EXPECT_DOUBLE_EQ(0, upperBound[20]) << "Joint 20 above upper bound " << q[20] << std::endl
                                 << "Upper Velocity Limits should be 0,"
                                 << upperBound[20] << " instead";
    EXPECT_LT(0, lowerBound[20]) << "Joint 20 at upper bound " << q[20] << std::endl
                                 << "Lower Velocity Limits should be < 0";
    */

    /*
    EXPECT_DOUBLE_EQ(0, upperBound[19]) << "Joint 19 at upper bound " << q[19] << std::endl
                                 << "Upper Velocity Limits should be 0,"
                                 << upperBound[19] << " instead";
    EXPECT_LT(0, lowerBound[19]) << "Joint 19 at upper bound " << q[19] << std::endl
                                 << "Lower Velocity Limits should be < 0";
    */

    EXPECT_DOUBLE_EQ(upperBound[22], -1*lowerBound[22]) << "Joint 22 in the middle of bounds "
                                                        << "[" << qLowerBounds[22] << "] <"
                                                        << q[22]
                                                        << "< [" << qUpperBounds[22] << "]" << std::endl
                                                        << "Lower and Upper Bounds Limits should be equal"
                                                        << " but are " << lowerBound[22]
                                                        << " and " << upperBound[22] << " respectively";
}

TEST_F(testJointLimits, boundsDoUpdate) {
    Eigen::VectorXd q(zeros.size()); q.setZero(q.size());
    Eigen::VectorXd q_next = Eigen::VectorXd::Constant(q.size(), 0.1);

    jointLimits->update(q);
    Eigen::VectorXd oldLowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd oldUpperBound = jointLimits->getUpperBound();

    jointLimits->update(q_next);

    Eigen::VectorXd newLowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd newUpperBound = jointLimits->getUpperBound();

    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
