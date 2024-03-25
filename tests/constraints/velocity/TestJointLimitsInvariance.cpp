#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/JointLimitsInvariace.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <xbot2_interface/xbotinterface2.h>
#include <cmath>
#include <matlogger2/matlogger2.h>

#include "../../common.h"


#define TORAD(deg) deg*M_PI/180.
#define _EPS_ 1e-5

namespace {

class testJointLimitsNaive : public TestBase {
 protected:

  testJointLimitsNaive() : TestBase("coman_floating_base")
  {
      _model_ptr->getJointLimits(qLowerBounds, qUpperBounds);

      q = _model_ptr->getNeutralQ();
      zeros = q;

      _model_ptr->setJointPosition(q);
      _model_ptr->update();


      vel_max.setOnes(_model_ptr->getNv());
      vel_max *= M_PI;

      dt = 0.001;

      jointLimits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr, qUpperBounds, qLowerBounds);

      jointVelocityLimits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, vel_max, dt);


      logger = XBot::MatLogger2::MakeLogger("/tmp/testJointLimitsNaive_velocity");
      logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

  }

  virtual ~testJointLimitsNaive() {

  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _model_ptr->setJointPosition(zeros);
      _model_ptr->update();
      jointLimits->update();
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for JointLimits.
public:

  OpenSoT::constraints::velocity::JointLimits::Ptr jointLimits;
  OpenSoT::constraints::velocity::VelocityLimits::Ptr jointVelocityLimits;

  Eigen::VectorXd qLowerBounds;
  Eigen::VectorXd qUpperBounds;
  Eigen::VectorXd zeros;
  Eigen::VectorXd vel_max;
  double dt;


  Eigen::VectorXd q;
  XBot::MatLogger2::Ptr logger;

};




// The fixture for testing class JointLimits.
class testJointLimits : public TestBase {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testJointLimits(): TestBase("coman_floating_base")
  {

      _model_ptr->getJointLimits(qLowerBounds, qUpperBounds);

      q = _model_ptr->getNeutralQ();
      zeros = q;

      _model_ptr->setJointPosition(q);
      _model_ptr->update();


      acc_max.setOnes(_model_ptr->getJointNum());
      acc_max *= 20.;

      vel_max.setOnes(_model_ptr->getNv());
      vel_max *= M_PI;

      dt = 0.001;

      jointLimitsInvariance = std::make_shared<OpenSoT::constraints::velocity::JointLimitsInvariance>(qUpperBounds, qLowerBounds, acc_max, *_model_ptr.get(), dt);
      jointLimitsInvariance->setPStepAheadPredictor(0.9);

      jointVelocityLimits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, vel_max, dt);


      logger = XBot::MatLogger2::MakeLogger("/tmp/testJointLimitsInvariace_velocity");
      logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

  }

  virtual ~testJointLimits() {

  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
      _model_ptr->setJointPosition(zeros);
      _model_ptr->update();
      jointLimitsInvariance->update();
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for JointLimits.
public:

  OpenSoT::constraints::velocity::JointLimitsInvariance::Ptr jointLimitsInvariance;
  OpenSoT::constraints::velocity::VelocityLimits::Ptr jointVelocityLimits;

  Eigen::VectorXd qLowerBounds;
  Eigen::VectorXd qUpperBounds;
  Eigen::VectorXd zeros;
  Eigen::VectorXd acc_max, vel_max;
  double dt;


  Eigen::VectorXd q;
  XBot::MatLogger2::Ptr logger;

};

TEST_F(testJointLimits, sizesAreCorrect) {
    unsigned int x_size = _model_ptr->getNv();

    Eigen::VectorXd lowerBound = jointLimitsInvariance->getLowerBound();
    Eigen::VectorXd upperBound = jointLimitsInvariance->getUpperBound();

    EXPECT_EQ(x_size, lowerBound.size()) << "lowerBound should have size"
                                         << x_size;
    EXPECT_EQ(x_size, upperBound.size()) << "upperBound should have size"
                                         << x_size;

    EXPECT_EQ(0, jointLimitsInvariance->getAeq().rows()) << "Aeq should have size 0"
                                               << "but has size"
                                               << jointLimitsInvariance->getAeq().rows();

    EXPECT_EQ(0, jointLimitsInvariance->getbeq().size()) << "beq should have size 0"
                                               << "but has size"
                                               << jointLimitsInvariance->getbeq().size();

    EXPECT_EQ(0,jointLimitsInvariance->getAineq().rows()) << "Aineq should have size 0"
                                                << "but has size"
                                                << jointLimitsInvariance->getAeq().rows();

    EXPECT_EQ(0,jointLimitsInvariance->getbLowerBound().size()) << "beq should have size 0"
                                                      << "but has size"
                                                      << jointLimitsInvariance->getbLowerBound().size();

    EXPECT_EQ(0,jointLimitsInvariance->getbUpperBound().size()) << "beq should have size 0"
                                                      << "but has size"
                                                      << jointLimitsInvariance->getbUpperBound().size();
}


TEST_F(testJointLimits, boundsDoUpdate) {
    Eigen::VectorXd q(zeros.size()); q.setZero(q.size());
    Eigen::VectorXd q_next = Eigen::VectorXd::Constant(q.size(), 0.05);

    jointLimitsInvariance->update();
    Eigen::VectorXd oldLowerBound = jointLimitsInvariance->getLowerBound();
    Eigen::VectorXd oldUpperBound = jointLimitsInvariance->getUpperBound();

    _model_ptr->setJointPosition(q_next);
    _model_ptr->update();

    jointLimitsInvariance->update();

    Eigen::VectorXd newLowerBound = jointLimitsInvariance->getLowerBound();
    Eigen::VectorXd newUpperBound = jointLimitsInvariance->getUpperBound();

    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testJointLimits, test_bounds)
{
    this->q = _model_ptr->getNeutralQ();

    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();


    OpenSoT::tasks::velocity::Postural::Ptr postural =
        std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);
    postural->setLambda(0.1);

    Eigen::VectorXd qref = 4. * Eigen::VectorXd::Ones(this->q.size());

    qref.segment(3, 4) = qref.segment(3, 4)/qref.segment(3, 4).norm();
//    qref[3] = 0.;
//    qref[4] = 0.;
//    qref[5] = 0.;
//    qref[6] = 1.;

    postural->setReference(qref);


    auto autostack = std::make_shared<OpenSoT::AutoStack>(postural);
    autostack<<this->jointLimitsInvariance<<this->jointVelocityLimits;

    auto solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    Eigen::VectorXd dq;
    Eigen::VectorXd qdot_prev;
    Eigen::VectorXd qdot, qddot;
    dq.setZero(this->_model_ptr->getNv());
    qdot_prev.setZero(this->_model_ptr->getNv());
    qdot.setZero(this->_model_ptr->getNv());
    qddot.setZero(this->_model_ptr->getNv());
    this->_model_ptr->setJointVelocity(qdot_prev);

    for(unsigned int i = 0; i < 3000; ++i)
    {
        this->logger->add("q", q);
        this->logger->add("qdot", qdot);
        this->logger->add("qddot", qddot);
        this->logger->add("qdot_prev", qdot_prev);
        this->logger->add("dq", dq);
        this->logger->add("qdot_lim", this->vel_max);
        this->logger->add("qddot_lim", this->acc_max);
        this->logger->add("qmin", this->qLowerBounds);
        this->logger->add("qmax", this->qUpperBounds);



        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->setJointVelocity(qdot_prev);
        this->_model_ptr->update();

        autostack->update();
        EXPECT_TRUE(solver->solve(dq));

        autostack->log(this->logger);


        this->q = _model_ptr->sum(this->q,  dq);
        qdot = dq/this->dt;



        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;

        Eigen::VectorXd _q = this->_model_ptr->difference(this->q, this->_model_ptr->getNeutralQ());

        for(unsigned int j = 0; j < this->qUpperBounds.size(); ++j)
        {
            if(_q[j] >= this->qUpperBounds[j])
                EXPECT_NEAR(_q[j] - this->qUpperBounds[j], 0.0, TORAD(0.01))<<"j:"<<j<<"  l:"<<this->qLowerBounds[j]<<"  u:"<<this->qUpperBounds[j]<<std::endl;
            EXPECT_LE(std::fabs(qdot[j]), this->vel_max[j]+_EPS_)<<"std::fabs(qdot[j]): "<<std::fabs(qdot[j])<<"   this->vel_max[j]:"<<this->vel_max[j]<<std::endl;
            EXPECT_LE(std::fabs(qddot[j]), this->acc_max[j]+_EPS_)<<"std::fabs(qddot[j]): "<<std::fabs(qddot[j])<<"   this->acc_max[j]:"<<this->acc_max[j]<<std::endl;
        }
    }


    qref = -4. * Eigen::VectorXd::Ones(this->q.size());
//    qref[3] = 0.;
//    qref[4] = 0.;
//    qref[5] = 0.;
//    qref[6] = 1.;

    qref.segment(3, 4) = qref.segment(3, 4)/qref.segment(3, 4).norm();
    postural->setReference(qref);

    for(unsigned int i = 0; i < 2000; ++i)
    {

        this->logger->add("q", q);
        this->logger->add("qdot", qdot);
        this->logger->add("qddot", qddot);
        this->logger->add("qdot_prev", qdot_prev);
        this->logger->add("dq", dq);
        this->logger->add("qdot_lim", this->vel_max);
        this->logger->add("qddot_lim", this->acc_max);
        this->logger->add("qmin", this->qLowerBounds);
        this->logger->add("qmax", this->qUpperBounds);

        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->setJointVelocity(qdot_prev);
        this->_model_ptr->update();

        autostack->update();
        EXPECT_TRUE(solver->solve(dq));

        autostack->log(this->logger);

        this->q = _model_ptr->sum(this->q,  dq);
        qdot = dq/this->dt;

        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;

        Eigen::VectorXd _q = this->_model_ptr->difference(this->q, this->_model_ptr->getNeutralQ());

        for(unsigned int j = 0; j < this->qLowerBounds.size(); ++j)
        {

            if(_q[j] <= this->qLowerBounds[j])
                EXPECT_NEAR(this->qLowerBounds[j] - _q[j], 0.0, TORAD(0.01))<<"j:"<<j<<"  l:"<<this->qLowerBounds[j]<<"  u:"<<this->qUpperBounds[j]<<std::endl;
            EXPECT_LE(std::fabs(qdot[j]), this->vel_max[j]+_EPS_)<<"std::fabs(qdot[j]): "<<std::fabs(qdot[j])<<"   this->vel_max[j]:"<<this->vel_max[j]<<std::endl;
            EXPECT_LE(std::fabs(qddot[j]), this->acc_max[j]+_EPS_)<<"std::fabs(qddot[j]): "<<std::fabs(qddot[j])<<"   this->acc_max[j]:"<<this->acc_max[j]<<std::endl;
        }

    }
}

/**
 * @brief TEST_F(testJointLimitsNaive, test_bounds) is only used for comparison, no tests are performed inside!
 */
TEST_F(testJointLimitsNaive, test_bounds)
{
    this->q = this->_model_ptr->getNeutralQ();

    this->_model_ptr->setJointPosition(this->q);
    this->_model_ptr->update();


    OpenSoT::tasks::velocity::Postural::Ptr postural =
        std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);

    Eigen::VectorXd qref = 4. * Eigen::VectorXd::Ones(this->q.size());

    qref.segment(3, 4) = qref.segment(3, 4)/qref.segment(3, 4).norm();

    postural->setReference(qref);


    auto autostack = std::make_shared<OpenSoT::AutoStack>(postural);
    autostack<<this->jointLimits<<this->jointVelocityLimits;

    auto solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    Eigen::VectorXd dq;
    Eigen::VectorXd qdot_prev;
    Eigen::VectorXd qdot, qddot;
    dq.setZero(this->_model_ptr->getNv());
    qdot_prev.setZero(this->_model_ptr->getNv());
    qdot.setZero(this->_model_ptr->getNv());
    qddot.setZero(this->_model_ptr->getNv());
    this->_model_ptr->setJointVelocity(qdot_prev);

    for(unsigned int i = 0; i < 3000; ++i)
    {
        this->logger->add("q", q);
        this->logger->add("qdot", qdot);
        this->logger->add("qddot", qddot);
        this->logger->add("qdot_prev", qdot_prev);
        this->logger->add("dq", dq);
        this->logger->add("qdot_lim", this->vel_max);
        this->logger->add("qmin", this->qLowerBounds);
        this->logger->add("qmax", this->qUpperBounds);



        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->setJointVelocity(qdot_prev);
        this->_model_ptr->update();

        autostack->update();
        EXPECT_TRUE(solver->solve(dq));

        autostack->log(this->logger);


        this->q = _model_ptr->sum(this->q,  dq);
        qdot = dq/this->dt;



        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;

    }


    qref = -4. * Eigen::VectorXd::Ones(this->q.size());
    qref.segment(3, 4) = qref.segment(3, 4)/qref.segment(3, 4).norm();
    postural->setReference(qref);

    for(unsigned int i = 0; i < 2000; ++i)
    {

        this->logger->add("q", q);
        this->logger->add("qdot", qdot);
        this->logger->add("qddot", qddot);
        this->logger->add("qdot_prev", qdot_prev);
        this->logger->add("dq", dq);
        this->logger->add("qdot_lim", this->vel_max);
        this->logger->add("qmin", this->qLowerBounds);
        this->logger->add("qmax", this->qUpperBounds);

        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->setJointVelocity(qdot_prev);
        this->_model_ptr->update();

        autostack->update();
        EXPECT_TRUE(solver->solve(dq));

        autostack->log(this->logger);

        this->q = _model_ptr->sum(this->q,  dq);
        qdot = dq/this->dt;

        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;


    }
}



}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
