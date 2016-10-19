#include <gtest/gtest.h>
#include <OpenSoT/legacy/constraints/velocity/JointLimits.h>
#include <idynutils/idynutils.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#define  s 1.0

using namespace yarp::math;

namespace {

// The fixture for testing class JointLimits.
class testJointLimits : public ::testing::Test {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testJointLimits()  :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
  {
    // You can do set-up work for each test here.

      qLowerBounds = coman.iDyn3_model.getJointBoundMin();
      qUpperBounds = coman.iDyn3_model.getJointBoundMax();
      zeros.resize(coman.iDyn3_model.getNrOfDOFs(),0.0);

      jointLimits = new OpenSoT::legacy::constraints::velocity::JointLimits(zeros,
                                    qUpperBounds,
                                    qLowerBounds);
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
      coman.updateiDyn3Model(zeros);
      jointLimits->update(zeros);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for JointLimits.

  iDynUtils coman;
  OpenSoT::legacy::constraints::velocity::JointLimits* jointLimits;

  yarp::sig::Vector qLowerBounds;
  yarp::sig::Vector qUpperBounds;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;
};

TEST_F(testJointLimits, sizesAreCorrect) {
    unsigned int x_size = coman.iDyn3_model.getNrOfDOFs();

    yarp::sig::Vector lowerBound = cartesian_utils::fromEigentoYarp(jointLimits->getLowerBound());
    yarp::sig::Vector upperBound = cartesian_utils::fromEigentoYarp(jointLimits->getUpperBound());

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

    yarp::sig::Vector q = zeros;
    q[16] = qLowerBounds[16] - 1E-1;
    q[17] = qLowerBounds[17];
    q[19] = qUpperBounds[19];
    q[20] = qUpperBounds[20] + 1E-1;

    q[22] = (qUpperBounds[22] + qLowerBounds[22])/2;
    q[23] = (qUpperBounds[22] + qLowerBounds[22])/2 - 1E-1;
    q[24] = (qUpperBounds[22] + qLowerBounds[22])/2 + 1E-1;

    coman.updateiDyn3Model(q);
    jointLimits->update(q);
    yarp::sig::Vector lowerBound = cartesian_utils::fromEigentoYarp(jointLimits->getLowerBound());
    yarp::sig::Vector upperBound = cartesian_utils::fromEigentoYarp(jointLimits->getUpperBound());

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
    yarp::sig::Vector q(zeros);
    yarp::sig::Vector q_next(zeros.size(), 0.1);

    jointLimits->update(q);
    yarp::sig::Vector oldLowerBound = cartesian_utils::fromEigentoYarp(jointLimits->getLowerBound());
    yarp::sig::Vector oldUpperBound = cartesian_utils::fromEigentoYarp(jointLimits->getUpperBound());

    jointLimits->update(q_next);

    yarp::sig::Vector newLowerBound = cartesian_utils::fromEigentoYarp(jointLimits->getLowerBound());
    yarp::sig::Vector newUpperBound = cartesian_utils::fromEigentoYarp(jointLimits->getUpperBound());

    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
