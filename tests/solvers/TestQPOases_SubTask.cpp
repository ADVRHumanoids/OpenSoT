#include <gtest/gtest.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/AutoStack.h>
#include "DefaultHumanoidStack.h"
#include <fstream>
#include <xbot2_interface/xbotinterface2.h>

#include "../common.h"



#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"


namespace {

class testQPOases_SubTask: public TestBase
{
protected:

    testQPOases_SubTask() : TestBase("coman_floating_base")
    {

    }

    virtual ~testQPOases_SubTask() {

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


TEST_F(testQPOases_SubTask, testSolveUsingSubTasks)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.);

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS.leftArm_Orientation / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds()));
    ASSERT_EQ(DHS.leftArm_Orientation->getTaskSize(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().cols(),_model_ptr->getNv());
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().cols(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getb().size(),3);

    OpenSoT::tasks::Aggregated::TaskPtr task = DHS.leftArm_Orientation;

    ASSERT_EQ(task->getTaskSize(),3);
    ASSERT_EQ(task->getA().rows(),3);
    ASSERT_EQ(task->getA().cols(),_model_ptr->getNv());
    ASSERT_EQ(task->getWeight().rows(),3);
    ASSERT_EQ(task->getWeight().cols(),3);
    ASSERT_EQ(task->getb().size(),3);
    ASSERT_EQ(task->getTaskSize(),3);

    for(unsigned int i = 0; i < 1000; ++i) {
        Eigen::MatrixXd H = task->getA().transpose() * task->getWeight() * task->getA();
        Eigen::VectorXd g = -1.0 * task->getLambda() * task->getA().transpose() * task->getWeight() * task->getb();

        ASSERT_EQ(H.rows(),_model_ptr->getNv());
        ASSERT_EQ(H.cols(),_model_ptr->getNv());
        ASSERT_EQ(g.size(),_model_ptr->getNv());
    }

    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    ASSERT_TRUE(solver->solve(dq));

    q = _model_ptr->sum(q, dq);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    subTaskTest->update(Eigen::VectorXd(0));
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveUsingSubTasksAndAggregated)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.);

    Eigen::VectorXd dq(_model_ptr->getNv());

    OpenSoT::AutoStack::Ptr subTaskTest = ((DHS.leftArm_Orientation + DHS.rightArm) / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds()));


    ASSERT_TRUE(solver->solve(dq));

    q = _model_ptr->sum(q, dq);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    subTaskTest->update(Eigen::VectorXd(0));
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveCartesianThroughSubTasksAndAggregated)
{

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.);

    OpenSoT::AutoStack::Ptr subTaskTest = ((DHS.leftArm_Position + DHS.leftArm_Orientation) / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds()));
    Eigen::MatrixXd actualPose = DHS.leftArm->getActualPose();
    Eigen::MatrixXd desiredPose = actualPose;
    desiredPose(0,3) = actualPose(0,3)+0.1;

    unsigned int iterations = 10000;
    while(sqrt(DHS.leftArm->getb().squaredNorm()) > 1e-4 && iterations > 0)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();
        subTaskTest->update(q);
        Eigen::VectorXd dq(_model_ptr->getNv()); dq.setZero();
        ASSERT_TRUE(solver->solve(dq));
        q = _model_ptr->sum(q, dq);
    }

    ASSERT_TRUE(sqrt(DHS.leftArm->getb().squaredNorm()) <= 1e-4);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
