#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cmath>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s

using namespace OpenSoT::constraints::velocity;
using namespace yarp::math;

namespace {

// The fixture for testing class CoMVelocity.
class testCoMVelocity : public ::testing::Test {
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testCoMVelocity() :
      coman("coman",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
  {
      std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
      std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

      _path_to_cfg = robotology_root + relative_path;

      _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
              (XBot::ModelInterface::getModel(_path_to_cfg));
      _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&coman, &null_deleter));

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      velocityLimits.resize(3,CoMVelocityLimit);
      zeros.resize(coman.iDynTree_model.getNrOfDOFs(),0.0);



      comVelocity = new CoMVelocity(conversion_utils_YARP::toEigen(velocityLimits),
                                    dT,
                                    conversion_utils_YARP::toEigen(zeros),
                                    *(_model_ptr.get()));
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
      comVelocity->update(conversion_utils_YARP::toEigen(zeros));
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
  XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
  std::string _path_to_cfg;
};

TEST_F(testCoMVelocity, sizesAreCorrect) {
    unsigned int x_size = coman.iDynTree_model.getNrOfDOFs();

    yarp::sig::Vector bLowerBound = conversion_utils_YARP::toYARP(comVelocity->getbLowerBound());
    yarp::sig::Vector bUpperBound = conversion_utils_YARP::toYARP(comVelocity->getbUpperBound());

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
    comVelocity->update(conversion_utils_YARP::toEigen(q));

    Aineq = conversion_utils_YARP::toYARP(comVelocity->getAineq());
    pAineq = pinv(Aineq);
    bLowerBound = conversion_utils_YARP::toYARP(comVelocity->getbLowerBound());
    bUpperBound = conversion_utils_YARP::toYARP(comVelocity->getbUpperBound());
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
        Eigen::MatrixXd _JCoM;
        coman.updateiDynTreeModel(conversion_utils_YARP::toEigen(qRight),true);
        coman.getCOMJacobian(_JCoM);
        yarp::sig::Matrix JCoM = conversion_utils_YARP::toYARP(_JCoM);
        JCoM = JCoM.removeCols(0,6);
        JCoM = JCoM.removeRows(3,3);
        qRight += pinv(JCoM) * dT * velocityLimits;
    }

    comVelocity->update(conversion_utils_YARP::toEigen(qRight));

    Aineq = conversion_utils_YARP::toYARP(comVelocity->getAineq());
    pAineq = pinv(Aineq);
    bLowerBound = conversion_utils_YARP::toYARP(comVelocity->getbLowerBound());
    bUpperBound = conversion_utils_YARP::toYARP(comVelocity->getbUpperBound());
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
        coman.updateiDynTreeModel(conversion_utils_YARP::toEigen(qLeft),true);
        coman.iDynTree_model.getCOMJacobian(JCoM);
        JCoM = JCoM.removeCols(0,6);
        JCoM = JCoM.removeRows(3,3);
        qLeft -= pinv(JCoM) * dT * velocityLimits;
    }

    comVelocity->update(conversion_utils_YARP::toEigen(qLeft));

    Aineq = conversion_utils_YARP::toYARP(comVelocity->getAineq());
    pAineq = pinv(Aineq);
    bLowerBound = conversion_utils_YARP::toYARP(comVelocity->getbLowerBound());
    bUpperBound = conversion_utils_YARP::toYARP(comVelocity->getbUpperBound());
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
