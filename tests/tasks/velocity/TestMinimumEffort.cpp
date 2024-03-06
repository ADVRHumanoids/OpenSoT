#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <memory>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/solvers/eHQP.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>
#include "../../common.h"



bool IS_ROSCORE_RUNNING;

namespace {

class testMinimumEffortTask: public TestBase
{

protected:
    testMinimumEffortTask() : TestBase("coman_floating_base")
    {


    }

    virtual ~testMinimumEffortTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testMinimumEffortTask, testMinimumEffortTask_)
{
    std::shared_ptr<ros::NodeHandle> _n;
    ros::Publisher joint_state_pub;
    if(IS_ROSCORE_RUNNING){
        _n.reset(new ros::NodeHandle());
        joint_state_pub = _n->advertise<sensor_msgs::JointState>("joint_states", 1000);
    }


    // setting initial position with bent legs
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();
    double angle = 45;
    q_whole[_model_ptr->getQIndex("RShSag")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RShLat")] = 0.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RElbj")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LShSag")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LShLat")] = 0.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LElbj")] = -angle*M_PI/180.0;

    if(IS_ROSCORE_RUNNING)
    {
        sensor_msgs::JointState joint_msg;
        for(unsigned int i = 1; i < _model_ptr->getJointNames().size(); ++i)
        {
            joint_msg.name.push_back(_model_ptr->getJointNames()[i]);
            joint_msg.position.push_back(q_whole[_model_ptr->getDofIndex(_model_ptr->getJointNames()[i])+1]);
            joint_msg.velocity.push_back(0.0);
            joint_msg.effort.push_back(0.0);
        }
        joint_msg.header.stamp = ros::Time::now();
    //    while(ros::ok())
    //        joint_state_pub.publish(joint_msg);
    }



    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    OpenSoT::tasks::velocity::MinimumEffort::Ptr minimumEffort;
    minimumEffort.reset(new OpenSoT::tasks::velocity::MinimumEffort(*_model_ptr));
    minimumEffort->update(Eigen::VectorXd(0));


    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack;
    stack.push_back(minimumEffort);

    OpenSoT::solvers::eHQP solver(stack);

    EXPECT_EQ(minimumEffort->getA().rows(), _model_ptr->getNv());
    EXPECT_EQ(minimumEffort->getb().size(), _model_ptr->getNv());

    EXPECT_TRUE(minimumEffort->getWeight().rows() == _model_ptr->getNv());
    EXPECT_TRUE(minimumEffort->getWeight().cols() == _model_ptr->getNv());

    EXPECT_TRUE(minimumEffort->getConstraints().size() == 0);

    double K = 1.;//0.8;
    minimumEffort->setLambda(K);
    EXPECT_DOUBLE_EQ(minimumEffort->getLambda(), K);

    Eigen::MatrixXd W(_model_ptr->getNv(), _model_ptr->getNv());
    W.setIdentity();
    minimumEffort->setW(1e-5*W);

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    Eigen::VectorXd tau_g;
    _model_ptr->computeGravityCompensation(tau_g);

    double initial_effort = tau_g.transpose()*minimumEffort->getW()*tau_g;
    std::cout<<"Initial Effort: "<<initial_effort<<std::endl;

    double initial_effort2 = minimumEffort->computeEffort();
    std::cout<<"Initial Effort2: "<<initial_effort2<<std::endl;

    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    for(unsigned int i = 0; i < 1250; ++i)
    {


        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        minimumEffort->update(Eigen::VectorXd(0));
        double old_effort = minimumEffort->computeEffort();


        solver.solve(dq);


        q_whole = _model_ptr->sum(q_whole, dq);



        if(IS_ROSCORE_RUNNING)
        {
            sensor_msgs::JointState joint_msg;
            for(unsigned int i = 1; i < _model_ptr->getJointNames().size(); ++i)
            {
                joint_msg.name.push_back(_model_ptr->getJointNames()[i]);
                joint_msg.position.push_back(q_whole[_model_ptr->getDofIndex(_model_ptr->getJointNames()[i])+1]);
            }
            joint_msg.header.stamp = ros::Time::now();
            joint_state_pub.publish(joint_msg);
        }




        minimumEffort->update(Eigen::VectorXd(0));
        EXPECT_LE(minimumEffort->computeEffort(), old_effort);
        std::cout << "Effort at step" << i << ": " << minimumEffort->computeEffort() << std::endl;

        if(IS_ROSCORE_RUNNING)
        {
            usleep(100);
            ros::spinOnce();
        }

    }
    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    _model_ptr->computeGravityCompensation(tau_g);

    double final_effort = tau_g.transpose()*minimumEffort->getW()*tau_g;
    EXPECT_LT(final_effort, initial_effort);
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "testMinimumEffort_node");
  IS_ROSCORE_RUNNING = ros::master::check();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
