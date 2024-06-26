#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <xbot2_interface/xbotinterface2.h>

#include "../../common.h"

namespace {

class testPosturalTask: public TestBase
{
protected:

    testPosturalTask() : TestBase("coman_floating_base")
    {

    }

    virtual ~testPosturalTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

void initializeIfNeeded()
{
    static bool is_initialized = false;

    if(!is_initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        srand((unsigned int)(seed));

        is_initialized = true;
    }

}

double getRandomAngle()
{
    initializeIfNeeded();
    return drand48()*2.0*M_PI-M_PI;
}

TEST_F(testPosturalTask, testPosturalTask_)
{
    std::cout<<"Is model floating_base: "<<_model_ptr->isFloatingBase()<<std::endl;

    Eigen::VectorXd q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();

    OpenSoT::tasks::velocity::Postural postural(*_model_ptr);
    std::cout<<"Postural Task Inited"<<std::endl;
    EXPECT_TRUE(postural.getA() == Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv()));
    EXPECT_TRUE(postural.getWeight() == Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv()));
    EXPECT_TRUE(postural.getConstraints().size() == 0);

    double K = 0.3;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    postural.setReference(q_ref);
    postural.update();
    EXPECT_TRUE(postural.getb() == postural.getLambda()*_model_ptr->difference(q_ref, q));

    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();
        postural.update();
        Eigen::VectorXd qq = postural.getb();
        q = _model_ptr->sum(q, qq);
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-3);
}

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

TEST_F(testPosturalTask, testPosturalTaskWithJointLimits_)
{
    std::cout<<"# JOINTS: "<<_model_ptr->getJointNum()<<std::endl;

    Eigen::VectorXd q(_model_ptr->getNq());
    q = _model_ptr->getNeutralQ();
    Eigen::VectorXd q_next = q;

    for(unsigned int i = 0; i < q.size(); ++i) {
        q[i] = getRandomAngle();
        q_next[i] = getRandomAngle();
        assert((q[i]!=q_next[i]));
    }
    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin,qmax);

    OpenSoT::tasks::velocity::Postural::TaskPtr postural( new OpenSoT::tasks::velocity::Postural(*_model_ptr) );
    OpenSoT::tasks::velocity::Postural::ConstraintPtr bound(
        new OpenSoT::constraints::velocity::JointLimits(*_model_ptr,
                        qmax,
                        qmin)
    );

    postural->getConstraints().push_back( bound );

    Eigen::VectorXd old_b = postural->getb();
    Eigen::VectorXd old_LowerBound = bound->getLowerBound();
    Eigen::VectorXd old_UpperBound = bound->getUpperBound();
    _model_ptr->setJointPosition(q_next);
    _model_ptr->update();
    postural->update();
    Eigen::VectorXd new_b = postural->getb();
    Eigen::VectorXd new_LowerBound = bound->getLowerBound();
    Eigen::VectorXd new_UpperBound = bound->getUpperBound();

    EXPECT_FALSE(old_b == new_b);
    EXPECT_FALSE(old_LowerBound == new_LowerBound);
    EXPECT_FALSE(old_UpperBound == new_UpperBound);

}

TEST_F(testPosturalTask, testReset)
{
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural postural(*_model_ptr);

    std::cout<<"INITIALIZATION: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    Eigen::VectorXd actual_q = postural.getActualPositions();
    std::cout<<"actual_q: \n"<<actual_q<<std::endl;
    Eigen::VectorXd reference_q = postural.getReference();
    std::cout<<"reference_q: \n"<<reference_q<<std::endl;
    std::cout<<"b: \n"<<postural.getb()<<std::endl;

    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
    {
        EXPECT_NEAR(postural.getb()[i], 0.0, 1e-9);
    }

    for(unsigned int i = 0; i < _model_ptr->getNq(); ++i)
    {
        EXPECT_EQ(actual_q[i], reference_q[i]);
    }

    std::cout<<"CHANGING q: ACTUAL AND REFERENCE DIFFERENT, b IS NOT 0"<<std::endl;
    q = _model_ptr->generateRandomQ();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    postural.update();

    actual_q = postural.getActualPositions();
    std::cout<<"actual_q: \n"<<actual_q<<std::endl;
    reference_q = postural.getReference();
    std::cout<<"reference_q: \n"<<reference_q<<std::endl;
    std::cout<<"b: \n"<<postural.getb()<<std::endl;

    std::cout<<"RESET: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    postural.reset();
    actual_q = postural.getActualPositions();
    std::cout<<"actual_q: \n"<<actual_q<<std::endl;
    reference_q = postural.getReference();
    std::cout<<"reference_q: \n"<<reference_q<<std::endl;
    std::cout<<"b: \n"<<postural.getb()<<std::endl;

    for(unsigned int i = 0; i < _model_ptr->getNv(); ++i)
    {
        EXPECT_NEAR(postural.getb()[i], 0.0, 1e-9);
    }

    for(unsigned int i = 0; i < _model_ptr->getNq(); ++i)
    {
        EXPECT_EQ(actual_q[i], reference_q[i]);
    }
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
