#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/CartesianPositionConstraint.h>
#include <idynutils/idynutils.h>
#include <idynutils/convex_hull.h>
#include <idynutils/tests_utils.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s
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
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
  {
    // You can do set-up work for each test here.

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.iDyn3_model.getNrOfDOFs(),0.0);
      coman.iDyn3_model.setFloatingBaseLink(coman.left_leg.index);
      _CartesianPositionConstraint = new CartesianPositionConstraint(  zeros, coman );
  }

  virtual ~testCartesianPositionConstraint() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(_CartesianPositionConstraint != NULL) {
        delete _CartesianPositionConstraint;
        _CartesianPositionConstraint = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _CartesianPositionConstraint->update(zeros);
      coman.updateiDyn3Model(zeros,true);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for CartesianPositionConstraint.

  iDynUtils coman;
  CartesianPositionConstraint* _CartesianPositionConstraint;

  yarp::sig::Vector velocityLimits;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;
};


TEST_F(testCartesianPositionConstraint, checkBoundsScaling) {
    // ------- Set The robot in a certain configuration ---------

    std::list<KDL::Vector> chPoints;
    coman.getSupportPolygonPoints(chPoints,"COM");

    idynutils::convex_hull idyn_convex_hull;
    std::vector<KDL::Vector> ch;
    idyn_convex_hull.getCartesianPositionConstraint(chPoints, ch);

    yarp::sig::Matrix A_ch;
    yarp::sig::Vector b_ch;
    yarp::sig::Matrix A_ch_1cm_scaling;
    yarp::sig::Vector b_ch_1cm_scaling;
    CartesianPositionConstraint::getConstraints(ch, A_ch, b_ch, 0.0);
    CartesianPositionConstraint::getConstraints(ch, A_ch_1cm_scaling, b_ch_1cm_scaling, 0.01);

    EXPECT_TRUE(A_ch == A_ch_1cm_scaling);

    for(unsigned int i = 0; i < b_ch.size(); ++i) {
        double norm_i = sqrt(A_ch(i,0)*A_ch(i,0) + A_ch(i,1)*A_ch(i,1));
        double distance_i = fabs(b_ch_1cm_scaling[i]-b_ch[i])/norm_i;
        EXPECT_DOUBLE_EQ(distance_i,.01);
    }
}

TEST_F(testCartesianPositionConstraint, sizesAreCorrect) {

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull huller;
    coman.getSupportPolygonPoints(points,"COM");
    huller.getCartesianPositionConstraint(points, ch);

    unsigned int hullSize = ch.size();

    unsigned int x_size = coman.iDyn3_model.getNrOfDOFs();

    EXPECT_EQ(0, _CartesianPositionConstraint->getLowerBound().size()) << "lowerBound should have size 0"
                                                      << "but has size"
                                                      <<  _CartesianPositionConstraint->getLowerBound().size();
    EXPECT_EQ(0, _CartesianPositionConstraint->getUpperBound().size()) << "upperBound should have size 0"
                                                      << "but has size"
                                                      << _CartesianPositionConstraint->getUpperBound().size();

    EXPECT_EQ(0, _CartesianPositionConstraint->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << _CartesianPositionConstraint->getAeq().rows();

    EXPECT_EQ(0, _CartesianPositionConstraint->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               <<  _CartesianPositionConstraint->getbeq().size();


    EXPECT_EQ(coman.iDyn3_model.getNrOfDOFs(),_CartesianPositionConstraint->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                                                              << coman.iDyn3_model.getNrOfDOFs()
                                                                              << " but has has "
                                                                              << _CartesianPositionConstraint->getAeq().cols()
                                                                              << " columns instead";

    EXPECT_EQ(0,_CartesianPositionConstraint->getbLowerBound().size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << _CartesianPositionConstraint->getbLowerBound().size();




    EXPECT_EQ(hullSize,_CartesianPositionConstraint->getAineq().rows()) << "Aineq should have size "
                                                       << hullSize
                                                       << " but has size"
                                                       << _CartesianPositionConstraint->getAineq().rows();


    EXPECT_EQ(hullSize,_CartesianPositionConstraint->getbUpperBound().size()) << "beq should have size "
                                                             << hullSize
                                                             << " but has size"
                                                             << _CartesianPositionConstraint->getbUpperBound().size();
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testCartesianPositionConstraint, BoundsAreCorrect) {

    // ------- Set The robot in a certain configuration ---------
    yarp::sig::Vector q(coman.iDyn3_model.getNrOfDOFs(), 0.0);
    q[coman.left_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.left_leg.joint_numbers[1]] = toRad(2.0);
    q[coman.left_leg.joint_numbers[2]] = toRad(-4.0);
    q[coman.left_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.left_leg.joint_numbers[4]] = toRad(-2.0);
    q[coman.left_leg.joint_numbers[5]] = toRad(-26.6);

    q[coman.right_leg.joint_numbers[0]] = toRad(-23.5);
    q[coman.right_leg.joint_numbers[1]] = toRad(-2.0);
    q[coman.right_leg.joint_numbers[2]] = toRad(0.0);
    q[coman.right_leg.joint_numbers[3]] = toRad(50.1);
    q[coman.right_leg.joint_numbers[4]] = toRad(2.0);
    q[coman.right_leg.joint_numbers[5]] = toRad(-26.6);

    updateiDyn3Model(true, q, coman);
    _CartesianPositionConstraint->update(q);

    // Get Vector of CH's points from coman
    std::list<KDL::Vector> points;
    coman.getSupportPolygonPoints(points,"COM");

    // Compute CH from previous points
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull huller;
    huller.getCartesianPositionConstraint(points, ch);

    //Compute CH from internal
    std::vector<KDL::Vector> ch2;
    _CartesianPositionConstraint->getCartesianPositionConstraint(ch2);

    std::cout << "CH:"<<std::endl;
    for(unsigned int i = 0; i < ch.size(); ++i)
        std::cout << ch[i].x() << " " << ch[i].y() << std::endl;

    std::cout << "CH2:"<<std::endl;
    for(unsigned int i = 0; i < ch2.size(); ++i)
        std::cout << ch2[i].x() << " " << ch2[i].y() << std::endl;

    ASSERT_EQ(ch.size(), ch2.size());
    for(unsigned int i = 0; i < ch.size(); ++i){
        ASSERT_DOUBLE_EQ(ch[i].x(), ch2[i].x());
        ASSERT_DOUBLE_EQ(ch[i].y(), ch2[i].y());
    }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
