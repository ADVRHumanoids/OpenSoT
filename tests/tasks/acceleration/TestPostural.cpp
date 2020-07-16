#include <gtest/gtest.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/utils/AutoStack.h>
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
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_floating_base.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

TEST_F(testPosturalTask, testPosturalTask_subtask)
{
    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero();
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::acceleration::Postural::Ptr postural(
                new OpenSoT::tasks::acceleration::Postural(*_model_ptr, q.size()));
    postural->update(q);

    std::cout<<"A size is: ["<<postural->getA().rows()<<" x "<<postural->getA().cols()<<"]"<<std::endl;
    std::cout<<"Model DoFs: "<<_model_ptr->getJointNum()<<std::endl;

    q.setZero(q.size());
    postural->setReference(q);
    postural->update(q);


    //std::list<unsigned int> idx = {_model_ptr->getDofIndex("WaistLat")};
    std::list<unsigned int> idx = {_model_ptr->getDofIndex("RWrj2")};

    std::cout<<"idx: "<<*(idx.begin())<<std::endl;
    OpenSoT::SubTask::Ptr sub_postural = postural%idx;

    sub_postural->update(q);

    std::cout<<"A task: "<<postural->getA()<<std::endl;
    std::cout<<"b task: "<<postural->getb()<<std::endl;

    std::cout<<"A subtask: "<<sub_postural->getA()<<std::endl;
    std::cout<<"b subtask: "<<sub_postural->getb()<<std::endl;


    EXPECT_EQ(sub_postural->getA().rows(), 1);
    for(unsigned int i = 0; i < sub_postural->getA().cols(); ++i)
    {
        if(i == *(idx.begin()))
            EXPECT_EQ(sub_postural->getA()(0,i), 1)<<"i: "<<i;
        else
            EXPECT_EQ(sub_postural->getA()(0,i), 0)<<"i: "<<i;
    }
}

TEST_F(testPosturalTask, testPosturalTask_)
{
    XBot::ModelInterface::Ptr _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);


    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    Eigen::VectorXd q0 = q;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::VectorXd q_ref(q.size()); q_ref.setZero();

    OpenSoT::tasks::acceleration::Postural postural(*_model_ptr, q_ref.size());
    postural.setReference(q_ref);
    std::cout<<"Postural Task Inited"<<std::endl;
    Eigen::MatrixXd A(q.size(), q.size());
    A.setIdentity();
    EXPECT_TRUE(postural.getA() == A);
    EXPECT_TRUE(postural.getWeight() == Eigen::MatrixXd::Identity(q.size(), q.size()))<<"W: \n"<<postural.getWeight()<<"\n"
        <<"W size: ["<<postural.getWeight().rows()<<" x "<<postural.getWeight().cols()<<"]"<<std::endl;

    double K = 1.;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    Eigen::VectorXd dq(q.size());
    dq.setZero(dq.size());
    double dT = 0.01;
    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(dq);
        _model_ptr->update();

        postural.update(q);
        Eigen::VectorXd ddq(q.size());
        ddq = postural.getb();
        dq += ddq*dT;
        q += dq*dT + 0.5*ddq*dT*dT;
    }

    for(unsigned int i = 6; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-5);

    std::cout<<"q0: "<<q0.transpose()<<std::endl;
    std::cout<<"q: "<<q.transpose()<<std::endl;
    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;

    EXPECT_TRUE(postural.checkConsistency());
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
