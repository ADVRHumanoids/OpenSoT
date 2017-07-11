#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <string>

namespace {

// The fixture for testing class Foo.
class testBilateralConstraint : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  testBilateralConstraint() {
    // You can do set-up work for each test here.
  }

  virtual ~testBilateralConstraint() {
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
TEST_F(testBilateralConstraint, BilateralConstraintWorks) {
    using namespace OpenSoT::constraints;
    const unsigned int nJ = 6;

    Eigen::VectorXd q(nJ); q.setZero(nJ);
    Eigen::VectorXd q_next(nJ); q_next = Eigen::VectorXd::Constant(nJ, M_PI-0.01);

    Eigen::MatrixXd A(nJ, nJ); A.setIdentity(nJ, nJ);

    Eigen::VectorXd bLowerBound(nJ); bLowerBound.setZero(nJ);
    Eigen::VectorXd bUpperBound(nJ);
    bUpperBound = Eigen::VectorXd::Constant(nJ, M_PI);

    BilateralConstraint::ConstraintPtr bilateral(BilateralConstraint::ConstraintPtr(
        new BilateralConstraint(A,bUpperBound,bLowerBound)));

    /* we should mash joint limits and velocity limits in one */
    EXPECT_TRUE(bilateral->getLowerBound().size() == 0);
    EXPECT_TRUE(bilateral->getUpperBound().size() == 0);
    /* we have a BilateralConstraint... */
    EXPECT_TRUE(bilateral->getAineq().rows() == nJ);
    EXPECT_TRUE(bilateral->getbLowerBound().size() == nJ);
    EXPECT_TRUE(bilateral->getbUpperBound().size() == nJ);
    /* and no equality constraint */
    EXPECT_TRUE(bilateral->getAeq().rows() == 0);
    EXPECT_TRUE(bilateral->getbeq().size() == 0);

    Eigen::VectorXd oldbLowerBound = bilateral->getbLowerBound();
    Eigen::VectorXd oldbUpperBound = bilateral->getbUpperBound();
    Eigen::MatrixXd oldAineq = bilateral->getAineq();
    bilateral->update(q_next);
    Eigen::VectorXd newbLowerBound = bilateral->getbLowerBound();
    Eigen::VectorXd newbUpperBound = bilateral->getbUpperBound();
    Eigen::MatrixXd newAineq = bilateral->getAineq();
    EXPECT_TRUE(oldbLowerBound == newbLowerBound);
    EXPECT_TRUE(oldbUpperBound == newbUpperBound);
    EXPECT_TRUE(oldAineq == newAineq);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
