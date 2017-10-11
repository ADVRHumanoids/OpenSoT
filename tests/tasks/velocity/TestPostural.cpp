#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <XBotInterface/ModelInterface.h>


namespace {

class testPosturalTask: public ::testing::Test
{
protected:

    testPosturalTask()
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
    Eigen::VectorXd q(6); q.setZero();
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    Eigen::VectorXd q_ref(q.size()); q_ref.setZero();

    OpenSoT::tasks::velocity::Postural postural(q);
    std::cout<<"Postural Task Inited"<<std::endl;
    EXPECT_TRUE(postural.getA() == Eigen::MatrixXd::Identity(q.size(), q.size()));
    EXPECT_TRUE(postural.getWeight() == Eigen::MatrixXd::Identity(q.size(), q.size()));
    EXPECT_TRUE(postural.getConstraints().size() == 0);

    double K = 0.1;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    postural.setReference(q_ref);
    postural.update(q);
    EXPECT_TRUE(postural.getb() == postural.getLambda()*(q_ref-q));

    for(unsigned int i = 0; i < 100; ++i)
    {
        postural.update(q);
        Eigen::VectorXd qq = postural.getb();
        q += qq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-3);
}

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT-lite/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

TEST_F(testPosturalTask, testPosturalTaskWithJointLimits_)
{
    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
    std::cout<<"# JOINTS: "<<_model_ptr->getJointNum()<<std::endl;

    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
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

    OpenSoT::tasks::velocity::Postural::TaskPtr postural( new OpenSoT::tasks::velocity::Postural(q) );
    OpenSoT::tasks::velocity::Postural::ConstraintPtr bound(
        new OpenSoT::constraints::velocity::JointLimits(q,
                        qmax,
                        qmin)
    );

    postural->getConstraints().push_back( bound );

    Eigen::VectorXd old_b = postural->getb();
    Eigen::VectorXd old_LowerBound = bound->getLowerBound();
    Eigen::VectorXd old_UpperBound = bound->getUpperBound();
    _model_ptr->setJointPosition(q_next);
    _model_ptr->update();
    postural->update(q_next);
    Eigen::VectorXd new_b = postural->getb();
    Eigen::VectorXd new_LowerBound = bound->getLowerBound();
    Eigen::VectorXd new_UpperBound = bound->getUpperBound();

    EXPECT_FALSE(old_b == new_b);
    EXPECT_FALSE(old_LowerBound == new_LowerBound);
    EXPECT_FALSE(old_UpperBound == new_UpperBound);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
