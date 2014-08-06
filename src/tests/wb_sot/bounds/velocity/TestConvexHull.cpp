#include <gtest/gtest.h>
#include <wb_sot/bounds/velocity/ConvexHull.h>
#include <drc_shared/idynutils.h>
#include <drc_shared/utils/convex_hull.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s

using namespace wb_sot::bounds::velocity;
using namespace yarp::math;

namespace {

// The fixture for testing class ConvexHull.
class testConvexHull : public ::testing::Test {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testConvexHull() {
    // You can do set-up work for each test here.

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.coman_iDyn3.getNrOfDOFs(),0.0);

      convexHull = new ConvexHull(  coman,
                                    coman.coman_iDyn3.getNrOfDOFs(),
                                    0.0);
  }

  virtual ~testConvexHull() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(convexHull != NULL) {
        delete convexHull;
        convexHull = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      convexHull->update(zeros);
      coman.updateiDyn3Model(zeros,zeros,zeros);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for ConvexHull.

  iDynUtils coman;
  ConvexHull* convexHull;

  yarp::sig::Vector velocityLimits;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;
};

TEST_F(testConvexHull, sizesAreCorrect) {

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    drc_shared::convex_hull huller;
    drc_shared::convex_hull::getSupportPolygonPoints(coman, points);
    huller.getConvexHull(points, ch);

    unsigned int hullSize = ch.size();

    unsigned int x_size = coman.coman_iDyn3.getNrOfDOFs();

    EXPECT_EQ(0, convexHull->getLowerBound().size()) << "lowerBound should have size 0"
                                                     << "but has size"
                                                     <<  convexHull->getLowerBound().size();
    EXPECT_EQ(0, convexHull->getUpperBound().size()) << "upperBound should have size 0"
                                                     << "but has size"
                                                     << convexHull->getUpperBound().size();

    EXPECT_EQ(0, convexHull->getAeq().rows()) << "Aeq should have size 0"
                                              << "but has size"
                                              << convexHull->getAeq().rows();

    EXPECT_EQ(0, convexHull->getbeq().size()) << "beq should have size 0"
                                              << "but has size"
                                              <<  convexHull->getbeq().size();


    EXPECT_EQ(2,convexHull->getAineq().cols()) <<  " Aineq should have number of columns equal to "
                                               << 2
                                               << " but has has "
                                               << convexHull->getAeq().cols()
                                               << " columns instead";

    EXPECT_EQ(0,convexHull->getbLowerBound().size()) << "beq should have size 3"
                                                     << "but has size"
                                                     << convexHull->getbLowerBound().size();




    EXPECT_EQ(hullSize,convexHull->getAineq().rows()) << "Aineq should have size "
                                                      << hullSize
                                                      << " but has size"
                                                      << convexHull->getAineq().rows();


    EXPECT_EQ(hullSize,convexHull->getbUpperBound().size()) << "beq should have size "
                                                            << hullSize
                                                            << " but has size"
                                                            << convexHull->getbUpperBound().size();
}

void getPointsFromConstraints(const yarp::sig::Matrix &A_ch,
                              const yarp::sig::Vector& b_ch,
                              std::vector<KDL::Vector>& points) {
    unsigned int nRects = A_ch.rows();

    for(unsigned int j = 0; j < nRects; j++) {
        unsigned int i = (j-1)%nRects;

        // get coefficients for i-th rect
        double a_i = A_ch(i,0);
        double b_i = A_ch(i,1);
        double c_i = -1*b_ch(i);

        // get coefficients for rect nect to i-th
        double a_j = A_ch(j,0);
        double b_j = A_ch(j,1);
        double c_j = -1*b_ch(j);

        /** Kramer rule to find intersection between two rects by Valerio Varricchio */
        double x = (-b_j*c_i+b_i*c_j)/(a_i*b_j-b_i*a_j);
        double y = (-a_i*c_j+c_i*a_j)/(a_i*b_j-b_i*a_j);
        points.push_back(KDL::Vector(x,y,0.0));
    }
}
// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testConvexHull, BoundsAreCorrect) {
    yarp::sig::Matrix Aineq = convexHull->getAineq();
    yarp::sig::Vector bUpperBound = convexHull->getbUpperBound();

    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch, chReconstructed;
    drc_shared::convex_hull huller;
    drc_shared::convex_hull::getSupportPolygonPoints(coman, points);
    huller.getConvexHull(points, ch);
    getPointsFromConstraints(Aineq, bUpperBound,
                             chReconstructed);

    std::cout << "CH:";
    for(unsigned int i = 0; i < ch.size(); ++i)
        std::cout << ch[i].x() << " " << ch[i].y() << std::endl;

    std::cout << "CH_RECONSTRUCTED:";
    for(unsigned int i = 0; i < chReconstructed.size(); ++i)
        std::cout << chReconstructed[i].x() << " " << chReconstructed[i].y() << std::endl;

    ASSERT_EQ(ch.size(),chReconstructed.size());

    try {
        for(unsigned int i = 0; i < ch.size(); ++i) {
            EXPECT_DOUBLE_EQ(ch[i].x(), chReconstructed[i].x()) << "ch.x and chReconstructed.x"
                                                                << " should be equal!" << std::endl;
            EXPECT_DOUBLE_EQ(ch[i].y(), chReconstructed[i].y()) << "ch.y and chReconstructed.y"
                                                                << " should be equal!" << std::endl;
        }
    } catch(...) {}
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
