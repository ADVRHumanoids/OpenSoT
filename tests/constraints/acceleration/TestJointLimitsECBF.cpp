#include <gtest/gtest.h>
#include <OpenSoT/constraints/acceleration/JointLimitsECBF.h>
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/logger.h>
#include <cmath>
#include <OpenSoT/solvers/iHQP.h>
#include <matlogger2/matlogger2.h>
#include "../../common.h"

namespace {

class testJointLimits : public TestBase {
protected:
    testJointLimits() : TestBase("coman_floating_base")
    {


        logger = XBot::MatLogger2::MakeLogger("/tmp/testJointLimitsECBF_acceleration");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    }

    void createStack()
    {
        qdot.setZero(_model_ptr->getNv());
        _model_ptr->setJointVelocity(qdot);


        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);

        _model_ptr->update();

        qddot = OpenSoT::AffineHelper::Identity(_model_ptr->getNv());


        acc_lims.setOnes(_model_ptr->getNv());
        acc_lims *= 20.;
        jointAccelerationLimits = std::make_shared<OpenSoT::constraints::GenericConstraint>(
                    "acceleration_limits", acc_lims, -acc_lims, acc_lims.size());


        dT = 0.001;
        qdotMax = M_PI;
        Eigen::VectorXd vel_lims;
        vel_lims.setOnes(_model_ptr->getNv());
        vel_lims *= qdotMax;

        jointVelocityLimits = std::make_shared<OpenSoT::constraints::acceleration::VelocityLimits>(
                    *_model_ptr, qddot, qdotMax, dT);

        _model_ptr->getJointLimits(qmin, qmax);
        jointLimits = std::make_shared<OpenSoT::constraints::acceleration::JointLimitsECBF>(
                    *_model_ptr, qddot, qmax, qmin, vel_lims, acc_lims);

        jointLimits->setAlpha1(15.);
        jointLimits->setAlpha2(15.);
        jointLimits->setAlpha3(15.);

        postural = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model_ptr, qddot);
        postural->setLambda(5000.);


        autostack = std::make_shared<OpenSoT::AutoStack>(postural);
        autostack<<jointLimits;//<<jointVelocityLimits;//<<jointAccelerationLimits;

        solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    }

    virtual ~testJointLimits() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
      // Code here will be called immediately after each test (right
      // before the destructor).
    }

    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model) {
        Eigen::VectorXd _q = _model_ptr->getNeutralQ();

        _q[_model->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getQIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model->getQIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model->getQIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model->getQIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model->getQIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model->getQIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model->getQIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model->getQIndex("RElbj")] = -80.0*M_PI/180.0;

        return _q;
    }

    void checkConstraints(const Eigen::VectorXd& qddot, double EPS)
    {

        Eigen::VectorXd _q = _model_ptr->difference(q, _model_ptr->getNeutralQ());

        for(unsigned int i = 0; i < qddot.size(); ++i)
        {
            double x = qddot[i];
            double x_min = jointAccelerationLimits->getLowerBound()[i];
            double x_max = jointAccelerationLimits->getUpperBound()[i];

            EXPECT_TRUE( (x-x_min >= -EPS) && (x-x_max <= EPS) )<<
            "joint acc violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;

            x_min = -qdotMax;
            x_max = qdotMax;
            x = qdot[i];
            EXPECT_TRUE( (x-x_min >= -EPS) && (x-x_max <= EPS) )<<
            "joint vel violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;

            EXPECT_TRUE( (_q[i]-qmin[i] >= -EPS) && (_q[i]-qmax[i] <= EPS) )<<
            "joint limits @i:"<<i<<" "<<qmin[i]<<" <= "<<_q[i]<<" <= "<<qmax[i]<<std::endl;

        }
    }


    OpenSoT::constraints::acceleration::JointLimitsECBF::Ptr jointLimits;
    OpenSoT::constraints::acceleration::VelocityLimits::Ptr jointVelocityLimits;
    OpenSoT::constraints::GenericConstraint::Ptr jointAccelerationLimits;

    OpenSoT::tasks::acceleration::Postural::Ptr postural;

    OpenSoT::AffineHelper qddot;

    OpenSoT::AutoStack::Ptr autostack;

    double dT;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    XBot::MatLogger2::Ptr logger;

    OpenSoT::solvers::iHQP::Ptr solver;

    double qdotMax;

    Eigen::VectorXd qmin, qmax;
    Eigen::VectorXd acc_lims;




};

TEST_F(testJointLimits, testBoundsWithTrajectory) {
    this->createStack();

    Eigen::VectorXd qref = _model_ptr->getNeutralQ();

    this->postural->setReference(qref);

    double T = 3;
    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update();
        //this->autostack->log(this->logger);

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));

        this->q = _model_ptr->sum(this->q, this->qdot*this->dT + 0.5*qddot*this->dT*this->dT);
        this->qdot += qddot*this->dT;


//        this->logger->add("qddot", qddot);
//        this->logger->add("qdot", qdot);
//        this->logger->add("q", q);
//        this->logger->add("qmax", qmax);
//        this->logger->add("qmin", qmin);
//        this->logger->add("qdotmax", this->qdotMax);
//        this->logger->add("qdotmin", -this->qdotMax);
//        this->logger->add("qddotmax", this->acc_lims);
//        this->logger->add("qddotmin", -this->acc_lims);
//        this->logger->add("qref", qref);

        this->checkConstraints(qddot, 1e-4);

    }

    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_LE(this->postural->getb()[i], 1e-6);


//     qref.setOnes(qref.size());
//     qref *= 10;

//     this->postural->setReference(qref);

    T = 10;
    Eigen::VectorXd q0 = this->q;
    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        qref.setOnes(qref.size());
        qref *= 3.* std::sin(i*this->dT);
        qref += q0;

        qref.segment(3,4) = qref.segment(3,4)/qref.segment(3,4).norm();

        this->postural->setReference(qref);

        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update();
        this->autostack->log(this->logger);

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));


        this->q = _model_ptr->sum(this->q, this->qdot*this->dT + 0.5*qddot*this->dT*this->dT);
        this->qdot += qddot*this->dT;

        this->logger->add("qddot", qddot);
        this->logger->add("qdot", qdot);
        this->logger->add("q", q);
        this->logger->add("qmax", qmax);
        this->logger->add("qmin", qmin);
        this->logger->add("qdotmax", this->qdotMax);
        this->logger->add("qdotmin", -this->qdotMax);
        this->logger->add("qddotmax", this->acc_lims);
        this->logger->add("qddotmin", -this->acc_lims);
        this->logger->add("qref", qref);

        this->checkConstraints(qddot, 1e-4);
    }

}

TEST_F(testJointLimits, testBoundsWithRegulation) {
    this->createStack();

    Eigen::VectorXd qref = _model_ptr->getNeutralQ();

    this->postural->setReference(qref);

    double T = 3;
    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update();

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));


        this->q = _model_ptr->sum(this->q, this->qdot*this->dT + 0.5*qddot*this->dT*this->dT);
        this->qdot += qddot*this->dT;

        this->checkConstraints(qddot, 1e-4);
    }

    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_LE(this->postural->getb()[i], 1e-6);

    auto logger = XBot::MatLogger2::MakeLogger("/tmp/testJointLimitsECBF_acceleration_regulation");
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    T = 7;
    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        qref.setOnes();

        if(i >= 0. && i < 2./this->dT)
            qref *= 3.;
        else if(i >= 2./this->dT && i < 3./this->dT)
            qref *= -3.;
        else if(i >= 3./this->dT && i < 4./this->dT)
            qref *= 3.;
        else if(i >= 4./this->dT && i < 5./this->dT)
            qref *= -3.;
        else if(i >= 5./this->dT && i < 6./this->dT)
            qref *= 3.;
        else
            qref *= -3.;

        qref.segment(3,4) = qref.segment(3,4)/qref.segment(3,4).norm();

        this->postural->setReference(qref);

        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update();
        this->autostack->log(this->logger);

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));


        this->q = this->_model_ptr->sum(this->q, this->qdot*this->dT + 0.5*qddot*this->dT*this->dT);
        this->qdot += qddot*this->dT;

        logger->add("qddot", qddot);
        logger->add("qdot", qdot);
        logger->add("q", q);
        logger->add("qmax", qmax);
        logger->add("qmin", qmin);
        logger->add("qdotmax", this->qdotMax);
        logger->add("qdotmin", -this->qdotMax);
        logger->add("qddotmax", this->acc_lims);
        logger->add("qddotmin", -this->acc_lims);
        logger->add("qref", qref);

        this->checkConstraints(qddot, 1e-4);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
