#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/JointLimitsInvariace.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <XBotInterface/ModelInterface.h>
#include <cmath>
#include <matlogger2/matlogger2.h>

std::string _path_to_cfg = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";

namespace {

// The fixture for testing class JointLimits.
class testJointLimits : public ::testing::Test {
 protected:

  // You can remove any or all of the following functions if its body
  // is empty.

  testJointLimits()
  {

      _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

      if(_model_ptr)
          std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
      else
          std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    // You can do set-up work for each test here.

      _model_ptr->getJointLimits(qLowerBounds, qUpperBounds);
      zeros.setZero(_model_ptr->getJointNum());
      q.setZero(_model_ptr->getJointNum());

      acc_max.setOnes(_model_ptr->getJointNum());
      acc_max *= 20.;

      vel_max.setOnes(_model_ptr->getJointNum());
      vel_max *= M_PI;

      dt = 0.001;

      jointLimitsInvariance = std::make_shared<OpenSoT::constraints::velocity::JointLimitsInvariance>(zeros,
                                    qUpperBounds, qLowerBounds, acc_max, *_model_ptr.get(), dt);

      jointVelocityLimits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(vel_max, dt);


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
      jointLimitsInvariance->update(zeros);
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
  XBot::ModelInterface::Ptr _model_ptr;
  XBot::MatLogger2::Ptr logger;

};

TEST_F(testJointLimits, sizesAreCorrect) {
    unsigned int x_size = _model_ptr->getJointNum();

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
    Eigen::VectorXd q_next = Eigen::VectorXd::Constant(q.size(), 0.1);

    jointLimitsInvariance->update(q);
    Eigen::VectorXd oldLowerBound = jointLimitsInvariance->getLowerBound();
    Eigen::VectorXd oldUpperBound = jointLimitsInvariance->getUpperBound();

    jointLimitsInvariance->update(q_next);

    Eigen::VectorXd newLowerBound = jointLimitsInvariance->getLowerBound();
    Eigen::VectorXd newUpperBound = jointLimitsInvariance->getUpperBound();

    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testJointLimits, test_bounds)
{
    this->q.setZero();

    this->_model_ptr->setJointPosition(this->q);


    OpenSoT::tasks::velocity::Postural::Ptr postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(q);

    Eigen::VectorXd qref = 4. * Eigen::VectorXd::Ones(this->q.size());
    postural->setReference(qref);


    auto autostack = std::make_shared<OpenSoT::AutoStack>(postural);
    autostack<<this->jointLimitsInvariance<<this->jointVelocityLimits;

    auto solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    Eigen::VectorXd dq;
    Eigen::VectorXd qdot_prev;
    Eigen::VectorXd qdot, qddot;
    dq.setZero(this->q.size());
    qdot_prev.setZero(this->q.size());
    qdot.setZero(this->q.size());
    qddot.setZero(this->q.size());
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

        autostack->update(q);
        EXPECT_TRUE(solver->solve(dq));


        this->q += dq;
        qdot = dq/this->dt;



        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;

//        for(unsigned int j = 0; j < this->qUpperBounds.size(); ++j)
//        {
//            if(this->q[j] >= this->qUpperBounds[j])
//                EXPECT_NEAR(this->q[j] - this->qUpperBounds[j], 0.0, 1e-5)<<"j:"<<j<<"  l:"<<this->qLowerBounds[j]<<"  u:"<<this->qUpperBounds[j]<<std::endl;

//            EXPECT_LE(std::fabs(qddot[j]), this->acc_max[j]);
//        }


    }

    qref = -4. * Eigen::VectorXd::Ones(this->q.size());
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

        autostack->update(q);
        EXPECT_TRUE(solver->solve(dq));

        this->q += dq;
        qdot = dq/this->dt;

        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->setJointVelocity(qdot);

        qddot = (qdot - qdot_prev)/this->dt;
        qdot_prev = qdot;

//        for(unsigned int j = 0; j < this->qUpperBounds.size(); ++j)
//        {

//            if(this->q[j] <= this->qLowerBounds[j])
//                EXPECT_NEAR(this->qLowerBounds[j] - this->q[j], 0.0, 1e-5)<<"j:"<<j<<"  l:"<<this->qLowerBounds[j]<<"  u:"<<this->qUpperBounds[j]<<std::endl;

//            EXPECT_LE(std::fabs(qddot[j]), this->acc_max[j]);
//        }

    }
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
