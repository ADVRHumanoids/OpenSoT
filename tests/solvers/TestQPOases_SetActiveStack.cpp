#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <OpenSoT/solvers/iHQP.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/utils/AutoStack.h>
#include "../common.h"


namespace {

class testActivateStack: public TestBase
{
protected:

    testActivateStack() : TestBase("coman_floating_base")
    {

    }

    virtual ~testActivateStack() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }


};

Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q = _model_ptr->getNeutralQ();
    _q[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")+1 ] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")+1 ] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")+1 ] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")+1 ] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")+1 ] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")+1 ] = -80.0*M_PI/180.0;

    return _q;
}

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
TEST_F(testActivateStack, test_deactivate_task)
{
    Eigen::VectorXd q = _model_ptr->generateRandomQ();

    //1) Test setActive with postural task
    OpenSoT::tasks::velocity::Postural::Ptr taskA(new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    taskA->update();
    Eigen::MatrixXd JA = taskA->getA();

    //here we deactivate the task
    taskA->setActive(false);
    taskA->update();

    EXPECT_TRUE(taskA->getA() == Eigen::MatrixXd::Zero(JA.rows(), JA.cols()));

    //here we reactivate the task
    taskA->setActive(true);
    taskA->update();

    EXPECT_TRUE(taskA->getA() == JA);

    //2) Test setActive with Cartesian task
    OpenSoT::tasks::velocity::Cartesian::Ptr taskB(new OpenSoT::tasks::velocity::Cartesian("foo", *_model_ptr,
                                                                                           "l_sole", "Waist"));
    taskB->update();
    Eigen::MatrixXd JB = taskB->getA();

    //here we deactivate the task
    taskB->setActive(false);
    taskB->update();

    EXPECT_TRUE(taskB->getA() == Eigen::MatrixXd::Zero(JB.rows(), JB.cols()));

    //here we reactivate the task
    taskB->setActive(true);
    taskB->update();

    EXPECT_TRUE(taskB->getA() == JB);

    //3) Test setActive with postural + Cartesian tasks
    OpenSoT::tasks::Aggregated::Ptr taskAB = (taskA + taskB);
    taskAB->update();
    Eigen::MatrixXd JAB = taskAB->getA();

    //here we deactivate the postural task
    taskA->setActive(false);
    taskAB->update();

    EXPECT_TRUE(taskAB->getA().block(0,0,JA.rows(),JA.cols()) == Eigen::MatrixXd::Zero(JA.rows(), JA.cols()));
    EXPECT_TRUE(taskAB->getA().block(JA.rows(),0,JB.rows(),JA.cols()) == JB);

    //here we reactivate the postural task and we deactivate the Cartesian task
    taskA->setActive(true);
    taskB->setActive(false);
    taskAB->update();

    EXPECT_TRUE(taskAB->getA().block(0,0,JA.rows(),JA.cols()) == JA);
    EXPECT_TRUE(taskAB->getA().block(JA.rows(),0,JB.rows(),JA.cols()) == Eigen::MatrixXd::Zero(JB.rows(), JB.cols()));

    //here both tasks are active
    taskA->setActive(true);
    taskB->setActive(true);
    taskAB->update();

    EXPECT_TRUE(taskAB->getA() == JAB);

    //here both tasks are not active
    taskA->setActive(false);
    taskB->setActive(false);
    taskAB->update();

    EXPECT_TRUE(taskAB->getA() == Eigen::MatrixXd::Zero(JA.rows() + JB.rows(), JB.cols()));

    //another way to have both the tasks deactiated using the Aggregated
    taskA->setActive(true);
    taskB->setActive(true);
    taskAB->setActive(false);
    taskAB->update();

    EXPECT_TRUE(taskAB->getA() == Eigen::MatrixXd::Zero(JA.rows() + JB.rows(), JB.cols()));

    //4)Check  Cartesian task with q update
    taskB->setActive(false);
    taskB->update();

    q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    taskB->update();
    EXPECT_TRUE(taskB->getA() == Eigen::MatrixXd::Zero(JB.rows(), JB.cols()));

    //The Jacobian is changed wrt the previous q!
    taskB->setActive(true);
    taskB->update();
    EXPECT_FALSE(taskB->getA() == JB);


}

TEST_F(testActivateStack, test_deactivate_stack)
{
    std::vector<Eigen::VectorXd> solutions;
    for(unsigned int j = 0; j <= 2; ++j)
    {
        Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        Cartesian::Ptr left_arm(new Cartesian("left_arm",   *_model_ptr, "LSoftHand", "Waist"));
        Cartesian::Ptr right_arm(new Cartesian("right_arm", *_model_ptr, "RSoftHand", "Waist"));
        Postural::Ptr postural(new Postural(*_model_ptr));

        Eigen::VectorXd q_min, q_max;
        _model_ptr->getJointLimits(q_min, q_max);
        JointLimits::Ptr joint_limits(new JointLimits(*_model_ptr, q_max, q_min));

        OpenSoT::AutoStack::Ptr autostack;
        if(j <= 1)
            autostack = ((left_arm)/
                     (right_arm)/
                     (postural)) << joint_limits;
        else
            autostack = ((left_arm)/
                     (postural)) << joint_limits;

        autostack->update();

        OpenSoT::solvers::iHQP sot(autostack->getStack(), autostack->getBounds());

        if(j == 1)
            sot.setActiveStack(1, false);

        KDL::Frame T;
        left_arm->getReference(T);
        T.p.x(T.p.x() + 0.1);
        left_arm->setReference(T);

        right_arm->getReference(T);
        T.p.x(T.p.x() - 0.1);
        right_arm->setReference(T);

        Eigen::VectorXd q_ref(q.size());
        q_ref = q;
        postural->setReference(q_ref);

        Eigen::VectorXd dq(_model_ptr->getNv());
        dq.setZero();
        for(unsigned int i = 0; i < 1000; ++i)
        {
            _model_ptr->setJointPosition(q);
            _model_ptr->update();

            autostack->update();

            ASSERT_TRUE(sot.solve(dq));

            q = _model_ptr->sum(q, dq);
        }

        solutions.push_back(q);
    }

    for(unsigned int k = 0; k < 3; ++k){
        Eigen::VectorXd qq = solutions[k];
        std::cout<<"q"<<k<<": "<<qq.transpose()<<std::endl;}

    for(unsigned int k = 0; k < solutions[0].size(); ++k)
        EXPECT_NEAR(solutions[1][k], solutions[2][k], 1e-10);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
