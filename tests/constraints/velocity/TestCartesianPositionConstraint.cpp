#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define toRad(X) (X * M_PI/180.0)

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace {

// The fixture for testing class CartesianPositionConstraint.
class testCartesianPositionConstraint : public ::testing::Test{
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testCartesianPositionConstraint() :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
      zeros(coman.getJointNames().size(),0.0),
      _DHS(coman, 3e-3, zeros),
      _A(1,3),
      _b(1,-0.5)
  {
    // You can do set-up work for each test here.

      coman.iDyn3_model.setFloatingBaseLink(coman.left_leg.index);
      // A,b represent a plane with normal along the z axis. We are imposing the z coordinate of the right_arm
      // should be greater than 0.5
      _A.zero(); _A(0,2) = -1.0;
      _cartesianPositionConstraint = new CartesianPositionConstraint(zeros, _DHS.rightArm, _A, _b);
  }

  virtual ~testCartesianPositionConstraint() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(_cartesianPositionConstraint != NULL) {
        delete _cartesianPositionConstraint;
        _cartesianPositionConstraint = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _cartesianPositionConstraint->update(zeros);
      coman.updateiDyn3Model(zeros,true);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for CartesianPositionConstraint.

  iDynUtils coman;
  yarp::sig::Vector zeros;

  CartesianPositionConstraint* _cartesianPositionConstraint;
  OpenSoT::DefaultHumanoidStack _DHS;

  yarp::sig::Vector q;
  yarp::sig::Matrix _A;
  yarp::sig::Vector _b;
};


TEST_F(testCartesianPositionConstraint, checkBoundsScaling) {
    // ------- Set The robot in a certain configuration ---------
    double bound = _cartesianPositionConstraint->getbUpperBound()[0];

    if(_cartesianPositionConstraint != NULL) {
      delete _cartesianPositionConstraint;
      _cartesianPositionConstraint = NULL;
    }
    _cartesianPositionConstraint = new CartesianPositionConstraint(zeros, _DHS.rightArm, _A, _b, 0.5);

    double boundSmall = _cartesianPositionConstraint->getbUpperBound()[0];


    EXPECT_DOUBLE_EQ(boundSmall,bound/2.0);
}

TEST_F(testCartesianPositionConstraint, sizesAreCorrect) {

    EXPECT_EQ(0, _cartesianPositionConstraint->getLowerBound().size()) << "lowerBound should have size 0"
                                                      << "but has size"
                                                      <<  _cartesianPositionConstraint->getLowerBound().size();
    EXPECT_EQ(0, _cartesianPositionConstraint->getUpperBound().size()) << "upperBound should have size 0"
                                                      << "but has size"
                                                      << _cartesianPositionConstraint->getUpperBound().size();

    EXPECT_EQ(0, _cartesianPositionConstraint->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << _cartesianPositionConstraint->getAeq().rows();

    EXPECT_EQ(0, _cartesianPositionConstraint->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               <<  _cartesianPositionConstraint->getbeq().size();


    EXPECT_EQ(coman.iDyn3_model.getNrOfDOFs(),_cartesianPositionConstraint->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << coman.iDyn3_model.getNrOfDOFs()
                                                                              << " but has has "
                                                                              << _cartesianPositionConstraint->getAeq().cols()
                                                                              << " columns instead";

    EXPECT_EQ(0,_cartesianPositionConstraint->getbLowerBound().size()) << "bLowerBound should have size 0"
                                                      << "but has size"
                                                      << _cartesianPositionConstraint->getbLowerBound().size();




    EXPECT_EQ(1,_cartesianPositionConstraint->getAineq().rows()) << "Aineq should have size "
                                                                 << 1
                                                                 << " but has size"
                                                                 << _cartesianPositionConstraint->getAineq().rows();


    EXPECT_EQ(1,_cartesianPositionConstraint->getbUpperBound().size()) << "bUpperBound should have size "
                                                                       << 1
                                                                       << " but has size"
                                                                       << _cartesianPositionConstraint->getbUpperBound().size();
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testCartesianPositionConstraint, BoundsAreCorrect) {

}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
