#include <gtest/gtest.h>
#include <OpenSoT/legacy/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/legacy/constraints/BilateralConstraint.h>
#include <OpenSoT/legacy/constraints/velocity/JointLimits.h>
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

    yarp::sig::Vector q(nJ, 0.0);
    yarp::sig::Vector q_next(nJ, M_PI - 0.01);

    yarp::sig::Matrix A(nJ,nJ); A.eye();
    yarp::sig::Vector bUpperBound(nJ,M_PI);
    yarp::sig::Vector bLowerBound(nJ,0.0);
    BilateralConstraint::ConstraintPtr bilateral(BilateralConstraint::ConstraintPtr(
        new BilateralConstraint(cartesian_utils::toEigen(A),
                                cartesian_utils::toEigen(bUpperBound),
                                cartesian_utils::toEigen(bLowerBound))
                                                  )
                          );

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

    yarp::sig::Vector oldbLowerBound = cartesian_utils::fromEigentoYarp(bilateral->getbLowerBound());
    yarp::sig::Vector oldbUpperBound = cartesian_utils::fromEigentoYarp(bilateral->getbUpperBound());
    yarp::sig::Matrix oldAineq = cartesian_utils::fromEigentoYarp(bilateral->getAineq());
    bilateral->update(cartesian_utils::toEigen(q_next));
    yarp::sig::Vector newbLowerBound = cartesian_utils::fromEigentoYarp(bilateral->getbLowerBound());
    yarp::sig::Vector newbUpperBound = cartesian_utils::fromEigentoYarp(bilateral->getbUpperBound());
    yarp::sig::Matrix newAineq = cartesian_utils::fromEigentoYarp(bilateral->getAineq());
    EXPECT_TRUE(oldbLowerBound == newbLowerBound);
    EXPECT_TRUE(oldbUpperBound == newbUpperBound);
    EXPECT_TRUE(oldAineq == newAineq);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
