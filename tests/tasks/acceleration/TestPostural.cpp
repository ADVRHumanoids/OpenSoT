#include <gtest/gtest.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
//#include <OpenSoT/constraints/velocity/JointLimits.h>
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

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT-lite/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

TEST_F(testPosturalTask, testPosturalTask_)
{
    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);


    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero();
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    Eigen::VectorXd q0 = q;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::VectorXd q_ref(q.size()); q_ref.setZero();

    OpenSoT::tasks::acceleration::Postural postural("postural",*_model_ptr, q_ref.size());
    postural.setReference(q_ref);
    std::cout<<"Postural Task Inited"<<std::endl;
    EXPECT_TRUE(postural.getA() == Eigen::MatrixXd::Identity(q.size(), q.size()));
    EXPECT_TRUE(postural.getWeight() == Eigen::MatrixXd::Identity(q.size(), q.size()));

    double K = 1.;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    Eigen::VectorXd dq(q.size());
    double dT = 0.01;
    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(dq);
        _model_ptr->update();

        postural.update(q);
        Eigen::VectorXd ddq = postural.getb();
        dq += ddq*dT;
        q += dq*dT + 0.5*ddq*dT*dT;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-5);

    std::cout<<"q0: "<<q0.transpose()<<std::endl;
    std::cout<<"q: "<<q.transpose()<<std::endl;
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
