#include <gtest/gtest.h>
#include <OpenSoT/constraints/acceleration/JointLimits.h>
#include <OpenSoT/constraints/acceleration/VelocityLimits.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Logger.hpp>
#include <cmath>
#include <OpenSoT/solvers/iHQP.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace {

class testJointLimits : public ::testing::Test {
protected:
    testJointLimits()
    {

        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

        logger = XBot::MatLogger::getLogger("/tmp/testJointLimits_acceleration");

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
    
        
        Eigen::VectorXd acc_lims(_model_ptr->getJointNum());
        acc_lims.setOnes(acc_lims.size());
        acc_lims *= 20.;
        jointAccelerationLimits = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                    "acceleration_limits", acc_lims, -acc_lims, acc_lims.size());


        dT = 0.01;
        qdotMax = M_PI;
        jointVelocityLimits = boost::make_shared<OpenSoT::constraints::acceleration::VelocityLimits>(
                    *_model_ptr, qddot, qdotMax, dT);

        _model_ptr->getJointLimits(qmin, qmax);
        jointLimits = boost::make_shared<OpenSoT::constraints::acceleration::JointLimits>(
                    *_model_ptr, qddot, qmax, qmin, acc_lims);

        postural = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model_ptr, qddot);
        postural->setLambda(1000.);


        autostack = boost::make_shared<OpenSoT::AutoStack>(postural);
        autostack<<jointLimits<<jointVelocityLimits<<jointAccelerationLimits;

        solver = boost::make_shared<OpenSoT::solvers::iHQP>(autostack->getStack(), autostack->getBounds(), 1e6);

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

    void checkConstraints(const Eigen::VectorXd& qddot)
    {

        for(unsigned int i = 0; i < qddot.size(); ++i)
        {
            double x = qddot[i];
            double x_min = jointAccelerationLimits->getLowerBound()[i];
            double x_max = jointAccelerationLimits->getUpperBound()[i];

//             EXPECT_TRUE((x >= x_min) && (x <= x_max));
//                     std::cout<<"acc violated @i "<<i<<" "<<x_min<<" <= "<<x<<" <= "x_max<<std::endl;

            ASSERT_LE(qdot[i], qdotMax)<<"Upper vel lim violated @i: "<<i<<std::endl; //check upper vel lim
            ASSERT_GE(qdot[i], -qdotMax)<<"Lower vel lim violated @i: "<<i<<std::endl; //check lower vel lim

//             EXPECT_LE(q[i], qmax[i])<<"Upper joint lim violated @i: "<<i<<std::endl;
//             EXPECT_GE(q[i], qmin[i])<<"Lower joint lim violated @i: "<<i<<std::endl;
        }
    }

    XBot::ModelInterface::Ptr _model_ptr;

    OpenSoT::constraints::acceleration::JointLimits::Ptr jointLimits;
    OpenSoT::constraints::acceleration::VelocityLimits::Ptr jointVelocityLimits;
    OpenSoT::constraints::GenericConstraint::Ptr jointAccelerationLimits;

    OpenSoT::tasks::acceleration::Postural::Ptr postural;

    OpenSoT::AffineHelper qddot;

    OpenSoT::AutoStack::Ptr autostack;

    double dT;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    XBot::MatLogger::Ptr logger;

    OpenSoT::solvers::iHQP::Ptr solver;

    double qdotMax;

    Eigen::VectorXd qmin, qmax;

};

TEST_F(testJointLimits, testBounds) {
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

        this->checkConstraints(qddot);
    }

    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_LE(this->postural->getb()[i], 1e-6);
    
    
    qref.setOnes();

    this->postural->setReference(10*qref);

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

        this->checkConstraints(qddot);
    }
    
    for(unsigned int i = 0; i < this->postural->getb().size(); ++i)
        EXPECT_NEAR(q[i], this->qmax[i], 1e-3);

    this->logger->flush();
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
