#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <xbot2_interface/xbotinterface2.h>
#include <cmath>
#define  s 1.0

#include "../../common.h"

std::string _path_to_cfg = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";

namespace {

// The fixture for testing class JointLimits.
class testJointLimits : public TestBase {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

     testJointLimits(): TestBase("coman")
  {


      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      _model_ptr->getJointLimits(qLowerBounds, qUpperBounds);
      zeros = _model_ptr->getNeutralQ();

      jointLimits = new OpenSoT::constraints::velocity::JointLimits(*_model_ptr, zeros,
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

  OpenSoT::constraints::velocity::JointLimits* jointLimits;

  Eigen::VectorXd qLowerBounds;
  Eigen::VectorXd qUpperBounds;
  Eigen::VectorXd zeros;
  Eigen::VectorXd q;
};

TEST_F(testJointLimits, sizesAreCorrect) {
    unsigned int x_size = _model_ptr->getNv();

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

    Eigen::VectorXd dq;
    dq.setZero(_model_ptr->getNv());

    dq[16] = qLowerBounds[16] - 1E-1;
    dq[17] = qLowerBounds[17];
    dq[19] = qUpperBounds[19];
    dq[20] = qUpperBounds[20] + 1E-1;

    dq[22] = (qUpperBounds[22] + qLowerBounds[22])/2;
    dq[23] = (qUpperBounds[22] + qLowerBounds[22])/2 - 1E-1;
    dq[24] = (qUpperBounds[22] + qLowerBounds[22])/2 + 1E-1;

    q = _model_ptr->sum(zeros, dq);

    _model_ptr->setJointPosition(_model_ptr->sum(zeros, dq));
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
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    Eigen::VectorXd q_next = _model_ptr->sum(q, Eigen::VectorXd::Constant(_model_ptr->getNv(), 0.1));

    jointLimits->update(q);
    Eigen::VectorXd oldLowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd oldUpperBound = jointLimits->getUpperBound();

    jointLimits->update(q_next);

    Eigen::VectorXd newLowerBound = jointLimits->getLowerBound();
    Eigen::VectorXd newUpperBound = jointLimits->getUpperBound();

    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testJointLimits, startingOutsideBoundsPositive)
{
    //Bounds between -1 and 1
    Eigen::VectorXd q_min;
    q_min = -Eigen::VectorXd::Ones(_model_ptr->getNv());
    Eigen::VectorXd q_max = -q_min;

    //q all zero but q[5] = 2
    Eigen::VectorXd q = zeros;
    q[5] = 2;  // ok since this model is euclidean

    OpenSoT::constraints::velocity::JointLimits joint_lims(*_model_ptr, q, q_max, q_min);
    joint_lims.update(q);

    Eigen::VectorXd lb = joint_lims.getLowerBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(lb[i], q_min[i] - q[i]);

    Eigen::VectorXd ub = joint_lims.getUpperBound();
    for(unsigned int i = 0; i < q.size(); ++i){
        if(i == 5)
            EXPECT_DOUBLE_EQ(ub[i], 0.0);
        else
            EXPECT_DOUBLE_EQ(ub[i], q_max[i] - q[i]);
    }

    q[5] = 0;
    joint_lims.update(q);
    lb = joint_lims.getLowerBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(lb[i], q_min[i] - q[i]);

    ub = joint_lims.getUpperBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(ub[i], q_max[i] - q[i]);
}

TEST_F(testJointLimits, startingOutsideBoundsNegative)
{
    //Bounds between -1 and 1
    Eigen::VectorXd q_min;
    q_min = -Eigen::VectorXd::Ones(_model_ptr->getNv());
    Eigen::VectorXd q_max = -q_min;

    //q all zero but q[5] = -2
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    q[5] = -2;

    OpenSoT::constraints::velocity::JointLimits joint_lims(*_model_ptr, q, q_max, q_min);
    joint_lims.update(q);

    Eigen::VectorXd lb = joint_lims.getLowerBound();
    for(unsigned int i = 0; i < q.size(); ++i){
        if(i == 5)
            EXPECT_DOUBLE_EQ(lb[i], 0.0);
        else
            EXPECT_DOUBLE_EQ(lb[i], q_min[i] - q[i]);}

    Eigen::VectorXd ub = joint_lims.getUpperBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(ub[i], q_max[i] - q[i]);

    q[5] = 0;
    joint_lims.update(q);
    lb = joint_lims.getLowerBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(lb[i], q_min[i] - q[i]);

    ub = joint_lims.getUpperBound();
    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_DOUBLE_EQ(ub[i], q_max[i] - q[i]);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
