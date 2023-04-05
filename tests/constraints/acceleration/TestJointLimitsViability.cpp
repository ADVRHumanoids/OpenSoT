#include <gtest/gtest.h>
#include <OpenSoT/constraints/acceleration/JointLimitsViability.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Logger.hpp>
#include <cmath>
#include <OpenSoT/solvers/iHQP.h>
#include <matlogger2/matlogger2.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

namespace {

class testJointLimitsViability : public ::testing::Test {
protected:
    testJointLimitsViability()
    {

        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

        logger = XBot::MatLogger2::MakeLogger("/tmp/testJointLimitsViability_acceleration");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    }

    void createStack()
    {
        qdot.setZero(_model_ptr->getJointNum());
        _model_ptr->setJointVelocity(qdot);
        this->logger->add("qdot", qdot);



        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        this->logger->add("q", q);

        _model_ptr->update();

        qddot = OpenSoT::AffineHelper::Identity(_model_ptr->getJointNum());


        acc_lims.setOnes(_model_ptr->getJointNum());
        acc_lims *= 20.;

        dT = 0.001;
        qdotMax = M_PI;
        Eigen::VectorXd vel_lims;
        vel_lims.setOnes(_model_ptr->getJointNum());
        vel_lims *= qdotMax;

        _model_ptr->getJointLimits(qmin, qmax);
        jointLimits = std::make_shared<OpenSoT::constraints::acceleration::JointLimitsViability>(
                    *_model_ptr, qddot, qmax, qmin, vel_lims, acc_lims, dT);

        postural = std::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model_ptr, qddot);
        postural->setLambda(1000.);


        autostack = std::make_shared<OpenSoT::AutoStack>(postural);
        autostack<<jointLimits;

        solver = std::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

    }

    virtual ~testJointLimitsViability() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
      // Code here will be called immediately after each test (right
      // before the destructor).
    }

    void checkConstraints(const Eigen::VectorXd& qddot, double EPS)
    {

        for(unsigned int i = 0; i < qddot.size(); ++i)
        {
            double x = qddot[i];
            double x_min = -acc_lims[i];
            double x_max = acc_lims[i];

            EXPECT_TRUE( (x-x_min >= -EPS) && (x-x_max <= EPS) )<<
            "joint acc violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;

            x_min = -qdotMax;
            x_max = qdotMax;
            x = qdot[i];
            EXPECT_TRUE( (x-x_min >= -EPS) && (x-x_max <= EPS) )<<
            "joint vel violated @i:"<<i<<" "<<x_min<<" <= "<<x<<" <= "<<x_max<<std::endl;

            EXPECT_TRUE( (q[i]-qmin[i] >= -EPS) && (q[i]-qmax[i] <= EPS) )<<
            "joint limits @i:"<<i<<" "<<qmin[i]<<" <= "<<q[i]<<" <= "<<qmax[i]<<std::endl;
        }
    }

    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model) {
        Eigen::VectorXd _q(_model->getJointNum());
        _q.setZero(_q.size());

        _q[_model->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
        _q[_model->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
        _q[_model->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
        _q[_model->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
        _q[_model->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

        return _q;
    }

    XBot::ModelInterface::Ptr _model_ptr;

    OpenSoT::constraints::acceleration::JointLimitsViability::Ptr jointLimits;

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

TEST_F(testJointLimitsViability, testBounds) {
    this->createStack();

    Eigen::VectorXd qref(this->q.size());
    qref.setZero();

    this->postural->setReference(qref);

    double T = 3;
    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update(Eigen::VectorXd(0));
        this->autostack->log(this->logger);

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));


        this->q += this->qdot*this->dT + 0.5*qddot*this->dT*this->dT;
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

    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_LE(this->postural->getb()[i], 1e-6);


//     qref.setOnes(qref.size());
//     qref *= 10;

//     this->postural->setReference(qref);

    T = 10;

    for(unsigned int  i = 0; i < T/this->dT; ++i)
    {
        qref.setOnes(qref.size());
        qref *= 10*std::sin(i*this->dT);
        this->postural->setReference(qref);

        this->_model_ptr->setJointVelocity(this->qdot);
        this->_model_ptr->setJointPosition(this->q);
        this->_model_ptr->update();

        this->autostack->update(Eigen::VectorXd(0));
        this->autostack->log(this->logger);

        Eigen::VectorXd qddot;
        ASSERT_TRUE(this->solver->solve(qddot));


        this->q += this->qdot*this->dT + 0.5*qddot*this->dT*this->dT;
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

    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_NEAR(q[i], this->qmax[i], 1e-4);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
