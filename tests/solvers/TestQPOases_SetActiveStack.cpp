#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <OpenSoT/solvers/QPOases.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/AutoStack.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace {

class testActivateStack: public ::testing::Test
{
protected:

    testActivateStack()
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

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;

TEST_F(testActivateStack, test_deactivate_task)
{
    std::vector<Eigen::VectorXd> solutions;
    XBot::ModelInterface::Ptr _model_ptr;
    for(unsigned int j = 0; j <= 2; ++j)
    {
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
        Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        Cartesian::Ptr left_arm(new Cartesian("left_arm", q, *_model_ptr, "LSoftHand", "Waist"));
        Cartesian::Ptr right_arm(new Cartesian("right_arm", q, *_model_ptr, "RSoftHand", "Waist"));
        Postural::Ptr postural(new Postural(q));

        Eigen::VectorXd q_min, q_max;
        _model_ptr->getJointLimits(q_min, q_max);
        JointLimits::Ptr joint_limits(new JointLimits(q, q_max, q_min));

        OpenSoT::AutoStack::Ptr autostack;
        if(j <= 1)
            autostack = ((left_arm)/
                     (right_arm)/
                     (postural)) << joint_limits;
        else
            autostack = ((left_arm)/
                     (postural)) << joint_limits;

        autostack->update(q);

        OpenSoT::solvers::QPOases_sot sot(autostack->getStack(), autostack->getBounds());

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
        q.setRandom(q.size());
        postural->setReference(q_ref);

        Eigen::VectorXd dq(q.size());
        dq.setZero(dq.size());
        for(unsigned int i = 0; i < 1000; ++i)
        {
            _model_ptr->setJointPosition(q);
            _model_ptr->update();

            autostack->update(q);

            ASSERT_TRUE(sot.solve(dq));

            q += dq;
        }

        solutions.push_back(q);
    }

    for(unsigned int k = 0; k < 3; ++k){
        Eigen::VectorXd qq = solutions[k];
        std::cout<<"q"<<k<<": "<<qq<<std::endl;}

//    for(unsigned int k = 0; k < solutions[0].size(); ++k)
//        EXPECT_NEAR(solutions[1][k], solutions[2][k], 1e-10);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
