#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/AutoStack.h>
#include "DefaultHumanoidStack.h"
#include <fstream>
#include <XBotInterface/ModelInterface.h>


#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

namespace {

class testQPOases_SubTask: public ::testing::Test
{
protected:

    testQPOases_SubTask()
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
    Eigen::VectorXd _q(_model_ptr->getJointNum());
    _q.setZero(_q.size());
    _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

    return _q;
}


TEST_F(testQPOases_SubTask, testSolveUsingSubTasks)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.,q);

    OpenSoT::AutoStack::Ptr subTaskTest = (DHS.leftArm_Orientation / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds()));
    ASSERT_EQ(DHS.leftArm_Orientation->getTaskSize(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getA().cols(),q.size());
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().rows(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getWeight().cols(),3);
    ASSERT_EQ(DHS.leftArm_Orientation->getb().size(),3);

    OpenSoT::tasks::Aggregated::TaskPtr task = DHS.leftArm_Orientation;

    ASSERT_EQ(task->getTaskSize(),3);
    ASSERT_EQ(task->getA().rows(),3);
    ASSERT_EQ(task->getA().cols(),q.size());
    ASSERT_EQ(task->getWeight().rows(),3);
    ASSERT_EQ(task->getWeight().cols(),3);
    ASSERT_EQ(task->getb().size(),3);
    ASSERT_EQ(task->getTaskSize(),3);

    for(unsigned int i = 0; i < 1000; ++i) {
        Eigen::MatrixXd H = task->getA().transpose() * task->getWeight() * task->getA();
        Eigen::VectorXd g = -1.0 * task->getLambda() * task->getA().transpose() * task->getWeight() * task->getb();

        ASSERT_EQ(H.rows(),q.size());
        ASSERT_EQ(H.cols(),q.size());
        ASSERT_EQ(g.size(),q.size());
    }

    Eigen::VectorXd dq(q.size());
    dq.setZero(dq.size());
    ASSERT_TRUE(solver->solve(dq));

    q += dq;
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    subTaskTest->update(q);
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveUsingSubTasksAndAggregated)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.,q);

    Eigen::VectorXd dq(q.size());

    OpenSoT::AutoStack::Ptr subTaskTest = ((DHS.leftArm_Orientation + DHS.rightArm) / DHS.postural)
                                            << DHS.jointLimits << DHS.velocityLimits;
    OpenSoT::solvers::iHQP::Ptr solver(
        new OpenSoT::solvers::iHQP(subTaskTest->getStack(), subTaskTest->getBounds()));


    ASSERT_TRUE(solver->solve(dq));

    q += dq;
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    subTaskTest->update(q);
    ASSERT_TRUE(solver->solve(dq));
}

TEST_F(testQPOases_SubTask, testSolveCartesianThroughSubTasksAndAggregated)
{
    XBot::ModelInterface::Ptr _model_ptr;
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
    _model_ptr->setJointPosition(q);
    _model_ptr->update();
    OpenSoT::DefaultHumanoidStack DHS(*_model_ptr, 1e-3,
                                      "Waist",
                                      "l_wrist", "r_wrist",
                                      "l_sole", "r_sole",
                                      1.,q);

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
        Eigen::VectorXd dq(q.size()); dq.setZero(dq.size());
        ASSERT_TRUE(solver->solve(dq));
        q += dq;
    }

    ASSERT_TRUE(sqrt(DHS.leftArm->getb().squaredNorm()) <= 1e-4);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
