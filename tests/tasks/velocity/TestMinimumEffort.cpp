#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <boost/make_shared.hpp>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/solvers/DampedPseudoInverse.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>


std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

bool IS_ROSCORE_RUNNING;

namespace {

class testMinimumEffortTask: public ::testing::Test
{

protected:
    XBot::ModelInterface::Ptr _model_ptr;
    int nJ;



    testMinimumEffortTask()
    {
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        nJ = _model_ptr->getJointNum();

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;





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
    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Publisher joint_state_pub;
    if(IS_ROSCORE_RUNNING){
        _n.reset(new ros::NodeHandle());
        joint_state_pub = _n->advertise<sensor_msgs::JointState>("joint_states", 1000);
    }


    // setting initial position with bent legs
    Eigen::VectorXd q_whole(nJ); q_whole.setZero(nJ);
    double angle = 45;
    q_whole[_model_ptr->getDofIndex("RShSag")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RShLat")] = 0.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RElbj")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LShSag")] = -angle*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LShLat")] = 0.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LElbj")] = -angle*M_PI/180.0;

    if(IS_ROSCORE_RUNNING)
    {
        sensor_msgs::JointState joint_msg;
        for(unsigned int i = 0; i < _model_ptr->getEnabledJointNames().size(); ++i)
        {
            joint_msg.name.push_back(_model_ptr->getEnabledJointNames()[i]);
            joint_msg.position.push_back(q_whole[_model_ptr->getDofIndex(_model_ptr->getEnabledJointNames()[i])]);
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
    minimumEffort.reset(new OpenSoT::tasks::velocity::MinimumEffort(q_whole, *(_model_ptr.get())));
    minimumEffort->update(q_whole);


    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack;
    stack.push_back(minimumEffort);

    OpenSoT::solvers::DampedPseudoInverse solver(stack);

    EXPECT_EQ(minimumEffort->getA().rows(), nJ);
    EXPECT_EQ(minimumEffort->getb().size(), nJ);

    EXPECT_TRUE(minimumEffort->getWeight().rows() == nJ);
    EXPECT_TRUE(minimumEffort->getWeight().cols() == nJ);

    EXPECT_TRUE(minimumEffort->getConstraints().size() == 0);

    double K = 1.;//0.8;
    minimumEffort->setLambda(K);
    EXPECT_DOUBLE_EQ(minimumEffort->getLambda(), K);

    Eigen::MatrixXd W(q_whole.size(), q_whole.size()); W.setIdentity(q_whole.size(), q_whole.size());
    minimumEffort->setW(1e-5*W);

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    Eigen::VectorXd tau_g;
    _model_ptr->computeGravityCompensation(tau_g);

    double initial_effort = tau_g.transpose()*minimumEffort->getW()*tau_g;
    std::cout<<"Initial Effort: "<<initial_effort<<std::endl;

    double initial_effort2 = minimumEffort->computeEffort();
    std::cout<<"Initial Effort2: "<<initial_effort2<<std::endl;

    Eigen::VectorXd dq(q_whole.size()); dq.setZero(q_whole.size());
    for(unsigned int i = 0; i < 1250; ++i)
    {


//        _model_ptr->setJointPosition(q_whole);
//        _model_ptr->update();

        minimumEffort->update(q_whole);
        double old_effort = minimumEffort->computeEffort();


        solver.solve(dq);


        q_whole += dq;



        if(IS_ROSCORE_RUNNING)
        {
            sensor_msgs::JointState joint_msg;
            for(unsigned int i = 0; i < _model_ptr->getEnabledJointNames().size(); ++i)
            {
                joint_msg.name.push_back(_model_ptr->getEnabledJointNames()[i]);
                joint_msg.position.push_back(q_whole[_model_ptr->getDofIndex(_model_ptr->getEnabledJointNames()[i])]);
            }
            joint_msg.header.stamp = ros::Time::now();
            joint_state_pub.publish(joint_msg);
        }




        minimumEffort->update(q_whole);
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
