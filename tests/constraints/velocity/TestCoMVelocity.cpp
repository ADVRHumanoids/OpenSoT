#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <XBotInterface/ModelInterface.h>
#include <cmath>
#define  s                1.0
#define  dT               0.001* s
#define  m_s              1.0
#define  CoMVelocityLimit 0.03 * m_s

using namespace OpenSoT::constraints::velocity;

namespace {

// The fixture for testing class CoMVelocity.
class testCoMVelocity : public ::testing::Test {
public:

protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testCoMVelocity()
  {
      std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
      //std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
      //std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";
      std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";

      _path_to_cfg = robotology_root + relative_path;

      _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      velocityLimits.resize(3);
      velocityLimits.setZero(3);
      velocityLimits << CoMVelocityLimit, CoMVelocityLimit, CoMVelocityLimit;

      zeros.setZero(_model_ptr->getJointNum());



      comVelocity = new CoMVelocity(velocityLimits,dT,zeros,*(_model_ptr.get()));
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

  CoMVelocity* comVelocity;

  Eigen::VectorXd velocityLimits;
  Eigen::VectorXd zeros;
  Eigen::VectorXd q;
  XBot::ModelInterface::Ptr _model_ptr;
  std::string _path_to_cfg;
};

TEST_F(testCoMVelocity, sizesAreCorrect) {
    unsigned int x_size = _model_ptr->getJointNum();

    Eigen::VectorXd bLowerBound = comVelocity->getbLowerBound();
    Eigen::VectorXd bUpperBound = comVelocity->getbUpperBound();

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

    Eigen::MatrixXd Aineq;
    Eigen::VectorXd bLowerBound;
    Eigen::VectorXd bUpperBound;
    Eigen::MatrixXd pAineq;
    // a q that causes a CoM velocity which is positive and smaller than velocityLimits
    Eigen::VectorXd qDotInPos;
    // a q that causes a CoM velocity which is negative and smaller than velocityLimits
    Eigen::VectorXd qDotInNeg;
    // a q that causes a CoM velocity which is positive and bigger than velocityLimits
    Eigen::VectorXd qDotOutPos;
    // a q that causes a CoM velocity which is positive and smaller than velocityLimits
    Eigen::VectorXd qDotOutNeg;

    Eigen::VectorXd q = zeros;
    comVelocity->update(q);

    Aineq = comVelocity->getAineq();
//    pAineq = pinv(Aineq);
    pAineq = Aineq.transpose()*(Aineq*Aineq.transpose()).inverse();
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

//    // testing bounds are correct
//    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT((Aineq*qDotInPos - velocityLimits).maxCoeff(),0.0) << "Aineq*qOk > b !!!";
//    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT((Aineq*qDotInNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qOk < -b !!!";
//    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT((Aineq*qDotOutPos - velocityLimits).minCoeff(),0.0) << "Aineq*qBad < b !!!";
//    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT((Aineq*qDotOutNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qBad > -b !!!";;


//    // configuration with CoM moved to the right
    Eigen::VectorXd qRight = q;

    _model_ptr->setJointPosition(qRight);
    _model_ptr->update();

    Eigen::Vector3d com_pose_init;
    _model_ptr->getCOM(com_pose_init);
    std::cout<<"CoM initial pose: "<<com_pose_init.transpose()<<std::endl;

//    // integrate 1s of moving right-down-backward
    for(unsigned int i = 0; i < 1/dT; ++i) {
        Eigen::MatrixXd JCoM;
        _model_ptr->setJointPosition(qRight);
        _model_ptr->update();

        _model_ptr->getCOMJacobian(JCoM);

//        JCoM = JCoM.removeCols(0,6);
//        JCoM = JCoM.removeRows(3,3);
//        qRight += pinv(JCoM) * dT * velocityLimits;


        qRight += JCoM.transpose()*(JCoM*JCoM.transpose()).inverse() * dT * (-velocityLimits);
    }

    _model_ptr->setJointPosition(qRight);
    _model_ptr->update();

    Eigen::Vector3d com_pose_final;
    _model_ptr->getCOM(com_pose_final);
    std::cout<<"CoM final pose: "<<com_pose_final.transpose()<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_LE(com_pose_final(i), com_pose_init(i));

    comVelocity->update(qRight);

    Aineq = comVelocity->getAineq();
//    pAineq = pinv(Aineq);
    pAineq = Aineq.transpose()*(Aineq*Aineq.transpose()).inverse();
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

    // testing bounds are correct
    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT((Aineq*qDotInPos - velocityLimits).maxCoeff(),0.0) << "Aineq*qOk > b !!!";
    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT((Aineq*qDotInNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qOk < -b !!!";
    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT((Aineq*qDotOutPos - velocityLimits).minCoeff(),0.0) << "Aineq*qBad < b !!!";
    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT((Aineq*qDotOutNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qBad > -b !!!";;

    // configuration with CoM moved to the left-up-forward
    Eigen::VectorXd qLeft = q;

    _model_ptr->setJointPosition(qLeft);
    _model_ptr->update();

    _model_ptr->getCOM(com_pose_init);
    std::cout<<"CoM initial pose: "<<com_pose_init.transpose()<<std::endl;


    // integrate 1s of moving left
    for(unsigned int i = 0; i < 1/dT; ++i) {
        Eigen::MatrixXd JCoM;

        _model_ptr->setJointPosition(qLeft);
        _model_ptr->update();

        _model_ptr->getCOMJacobian(JCoM);


        qLeft += JCoM.transpose()*(JCoM*JCoM.transpose()).inverse() * dT * velocityLimits;

    }

    _model_ptr->setJointPosition(qLeft);
    _model_ptr->update();

    _model_ptr->getCOM(com_pose_final);
    std::cout<<"CoM final pose: "<<com_pose_final.transpose()<<std::endl;

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_LE(com_pose_init(i), com_pose_final(i));

    comVelocity->update(qLeft);

    Aineq = comVelocity->getAineq();
//////    pAineq = pinv(Aineq);
    pAineq = Aineq.transpose()*(Aineq*Aineq.transpose()).inverse();
    bLowerBound = comVelocity->getbLowerBound();
    bUpperBound = comVelocity->getbUpperBound();
    qDotInPos = pAineq * 0.5 * velocityLimits;
    qDotInNeg = pAineq * -0.5 * velocityLimits;
    qDotOutPos = pAineq * 1.5 * velocityLimits;
    qDotOutNeg = pAineq * -1.5 * velocityLimits;

    // testing bounds are correct
    /** Aq < b => Aq - b < 0 => max(Aq-b < 0)*/
    EXPECT_LT((Aineq*qDotInPos - velocityLimits).maxCoeff(),0.0) << "Aineq*qOk > b !!!";
    /** -b < Aq => Aq + b > 0 => min(Aq+b > 0)*/
    EXPECT_GT((Aineq*qDotInNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qOk < -b !!!";
    /** Aq > b => Aq - b > 0 => min(Aq-b > 0)*/
    EXPECT_GT((Aineq*qDotOutPos - velocityLimits).minCoeff(),0.0) << "Aineq*qBad < b !!!";
    /** -b > Aq => Aq + b < 0 => max(Aq+b < 0)*/
    EXPECT_LT((Aineq*qDotOutNeg + velocityLimits).minCoeff(),0.0) << "Aineq*qBad > -b !!!";;

}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
