#include <gtest/gtest.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/utils/AutoStack.h>
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

TEST_F(testPosturalTask, testPosturalTask_subtask)
{
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::acceleration::Postural::Ptr postural(
                new OpenSoT::tasks::acceleration::Postural(*_model_ptr));
    postural->update(Eigen::VectorXd(0));

    std::cout<<"A size is: ["<<postural->getA().rows()<<" x "<<postural->getA().cols()<<"]"<<std::endl;
    std::cout<<"Model DoFs: "<<_model_ptr->getJointNum()<<std::endl;

    q = _model_ptr->getNeutralQ();
    postural->setReference(q);
    postural->update(Eigen::VectorXd(0));


    //std::list<unsigned int> idx = {_model_ptr->getDofIndex("WaistLat")};
    std::list<unsigned int> idx = {_model_ptr->getVIndex("RWrj2")};

    std::cout<<"idx: "<<*(idx.begin())<<std::endl;
    OpenSoT::SubTask::Ptr sub_postural = postural%idx;

    sub_postural->update(Eigen::VectorXd(0));

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
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    Eigen::VectorXd q0 = q;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    Eigen::VectorXd q_ref = _model_ptr->getNeutralQ();

    OpenSoT::tasks::acceleration::Postural postural(*_model_ptr);
    postural.setReference(q_ref);
    std::cout<<"Postural Task Inited"<<std::endl;
    Eigen::MatrixXd A(_model_ptr->getNv(), _model_ptr->getNv());
    A.setIdentity();
    EXPECT_TRUE(postural.getA() == A);
    EXPECT_TRUE(postural.getWeight() == Eigen::MatrixXd::Identity(_model_ptr->getNv(), _model_ptr->getNv()))<<"W: \n"<<postural.getWeight()<<"\n"
        <<"W size: ["<<postural.getWeight().rows()<<" x "<<postural.getWeight().cols()<<"]"<<std::endl;

    double K = 1.;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    double dT = 0.01;
    for(unsigned int i = 0; i < 10000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->setJointVelocity(dq);
        _model_ptr->update();

        postural.update(Eigen::VectorXd(0));
        Eigen::VectorXd ddq(q.size());
        ddq = postural.getb();
        dq += ddq*dT;
        q = _model_ptr->sum(q, dq*dT + 0.5*ddq*dT*dT);
    }

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-5);

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
