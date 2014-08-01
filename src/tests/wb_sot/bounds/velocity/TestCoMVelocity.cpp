#include <gtest/gtest.h>
#include <wb_sot/bounds/velocity/CoMVelocity.h>
#include <drc_shared/idynutils.h>
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

// The fixture for testing class CoMVelocity.
class testCoMVelocity : public ::testing::Test {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testCoMVelocity() {
    // You can do set-up work for each test here.

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.coman_iDyn3.getNrOfDOFs(),0.0);

      comVelocity = new CoMVelocity(velocityLimits,
                                    coman,
                                    dT,
                                    coman.coman_iDyn3.getNrOfDOFs());
  }

  virtual ~testCoMVelocity() {
    // You can do clean-up work that doesn't throw exceptions here.
      if(comVelocity != NULL) {
        delete comVelocity;
        comVelocity = NULL;
      }
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      comVelocity->update(zeros);
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for CoMVelocity.

  iDynUtils coman;
  CoMVelocity* comVelocity;

  yarp::sig::Vector velocityLimits;
  yarp::sig::Vector zeros;
  yarp::sig::Vector q;
};

TEST_F(testCoMVelocity, sizesAreCorrect) {
    unsigned int x_size = coman.coman_iDyn3.getNrOfDOFs();

    yarp::sig::Vector bLowerBound = comVelocity->getbLowerBound();
    yarp::sig::Vector bUpperBound = comVelocity->getbUpperBound();

    EXPECT_EQ(0, comVelocity->getLowerBound().size()) << "lowerBound should have size 0"
                                                        << "but has size"
                                                        <<  comVelocity->getLowerBound().size();
    EXPECT_EQ(0, comVelocity->getUpperBound().size()) << "upperBound should have size 0"
                                                        << "but has size"
                                                        << comVelocity->getUpperBound().size();

    EXPECT_EQ(0, comVelocity->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << comVelocity->getAeq().rows();

    EXPECT_EQ(0, comVelocity->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               <<  comVelocity->getbeq().size();

    EXPECT_EQ(3,comVelocity->getAineq().rows()) << "Aineq should have size 3"
                                                << "but has size"
                                                << comVelocity->getAeq().rows();

    EXPECT_EQ(x_size,comVelocity->getAineq().cols())<< "Aineq should have number of columns equal to "
                                                    << x_size
                                                    << " but has has "
                                                    << comVelocity->getAeq().cols()
                                                    << " columns instead";

    EXPECT_EQ(3,bLowerBound.size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << comVelocity->getbLowerBound().size();

    EXPECT_EQ(3,bUpperBound.size()) << "beq should have size 3"
                                                      << "but has size"
                                                      << comVelocity->getbUpperBound().size();
}

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testCoMVelocity, BoundsAreCorrect) {

    yarp::sig::Matrix Aineq;
    yarp::sig::Vector bLowerBound;
    yarp::sig::Vector bUpperBound;
    yarp::sig::Matrix pAineq;
    // a q that causes a CoM velocity which is positive and smaller than velocityLimits
    yarp::sig::Vector qDotInPos;
    // a q that causes a CoM velocity which is negative and smaller than velocityLimits
    yarp::sig::Vector qDotInNeg;
    // a q that causes a CoM velocity which is positive and bigger than velocityLimits
    yarp::sig::Vector qDotOutPos;
    // a q that causes a CoM velocity which is positive and smaller than velocityLimits
    yarp::sig::Vector qDotOutNeg;

    yarp::sig::Vector q = zeros;
    comVelocity->update(q);

    Aineq = comVelocity->getAineq();
    pAineq = pinv(Aineq);
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

    // testing bounds are correct
    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT(findMax(Aineq*qDotInPos - velocityLimits),0.0) << "Aineq*qOk > b !!!";
    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotInNeg + velocityLimits),0.0) << "Aineq*qOk < -b !!!";
    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotOutPos - velocityLimits),0.0) << "Aineq*qBad < b !!!";
    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT(findMin(Aineq*qDotOutNeg + velocityLimits),0.0) << "Aineq*qBad > -b !!!";;


    // configuration with CoM moved to the right
    yarp::sig::Vector qRight = q;

    // integrate 1s of moving right
    for(unsigned int i = 0; i < 1/dT; ++i) {
        yarp::sig::Matrix JCoM;
        coman.updateiDyn3Model(qRight,zeros,zeros);
        coman.coman_iDyn3.getCOMJacobian(JCoM);
        JCoM = JCoM.removeCols(0,6);
        JCoM = JCoM.removeRows(3,3);
        qRight += pinv(JCoM) * dT * velocityLimits;
    }

    comVelocity->update(qRight);

    Aineq = comVelocity->getAineq();
    pAineq = pinv(Aineq);
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

    // testing bounds are correct
    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT(findMax(Aineq*qDotInPos - velocityLimits),0.0) << "Aineq*qOk > b !!!";
    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotInNeg + velocityLimits),0.0) << "Aineq*qOk < -b !!!";
    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotOutPos - velocityLimits),0.0) << "Aineq*qBad < b !!!";
    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT(findMin(Aineq*qDotOutNeg + velocityLimits),0.0) << "Aineq*qBad > -b !!!";;

    // configuration with CoM moved to the left
    yarp::sig::Vector qLeft = q;

    // integrate 1s of moving left
    for(unsigned int i = 0; i < 1/dT; ++i) {
        yarp::sig::Matrix JCoM;
        coman.updateiDyn3Model(qLeft,zeros,zeros);
        coman.coman_iDyn3.getCOMJacobian(JCoM);
        JCoM = JCoM.removeCols(0,6);
        JCoM = JCoM.removeRows(3,3);
        qLeft -= pinv(JCoM) * dT * velocityLimits;
    }

    comVelocity->update(qLeft);

    Aineq = comVelocity->getAineq();
    pAineq = pinv(Aineq);
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

    // testing bounds are correct
    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT(findMax(Aineq*qDotInPos - velocityLimits),0.0) << "Aineq*qOk > b !!!";
    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotInNeg + velocityLimits),0.0) << "Aineq*qOk < -b !!!";
    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT(findMin(Aineq*qDotOutPos - velocityLimits),0.0) << "Aineq*qBad < b !!!";
    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT(findMin(Aineq*qDotOutNeg + velocityLimits),0.0) << "Aineq*qBad > -b !!!";;

}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
