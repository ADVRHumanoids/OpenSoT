#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <memory>
#include <OpenSoT/utils/cartesian_utils.h>
#include <xbot2_interface/xbotinterface2.h>
#include "../../common.h"


namespace {

class testCartesianTask: public TestBase
{
protected:

    testCartesianTask(): TestBase("coman_floating_base")
    {

    }

    virtual ~testCartesianTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testCartesianTask, testSetBaseLink)
{
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    q[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;
    q[_model_ptr->getQIndex("LShSag")] =  20.0*M_PI/180.0;
    q[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    q[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::string distal_link = "l_wrist";
    std::string base_link1 = "world";
    std::string base_link2 = "l_sole";
    std::string base_link3 = "Waist";

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm1(
        new OpenSoT::tasks::velocity::Cartesian("l_arm", *_model_ptr,distal_link,
                                                base_link1));
    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm2(
        new OpenSoT::tasks::velocity::Cartesian("l_arm",*_model_ptr,distal_link,
                                                base_link2));
    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm3(
        new OpenSoT::tasks::velocity::Cartesian("l_arm",*_model_ptr,distal_link,
                                                base_link3));

    l_arm1->update();
    l_arm2->update();
    l_arm3->update();

    EXPECT_TRUE(l_arm1->setBaseLink(base_link2));
    l_arm1->update();

    EXPECT_NEAR((l_arm1->getA() - l_arm2->getA()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getb() - l_arm2->getb()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getReference() - l_arm2->getReference()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getActualPose() - l_arm2->getActualPose()).norm(), 0.0, 1e-12);

    EXPECT_TRUE(l_arm1->setBaseLink(base_link3));
    l_arm1->update();

    EXPECT_NEAR((l_arm1->getA() - l_arm3->getA()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getb() - l_arm3->getb()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getReference() - l_arm3->getReference()).norm(), 0.0, 1e-12);
    EXPECT_NEAR((l_arm1->getActualPose() - l_arm3->getActualPose()).norm(), 0.0, 1e-12);
}


TEST_F(testCartesianTask, testCartesianTaskWorldGlobal_)
{
    // setting initial position with bent legs
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();

    q_whole[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::right_leg",
                                                 *_model_ptr,
                                                 "r_sole",
                                                 "world");

    // setting x_ref with a delta offset along the z axis (-2cm)
    Eigen::MatrixXd delta_x(4,4); delta_x.setZero(4,4);
    delta_x(2,3) = -0.02;
    Eigen::Affine3d x;
    _model_ptr->getPose("r_sole", x);

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x.matrix()<<std::endl;
    std::cout<<"delta_x: "<<delta_x<<std::endl;

    Eigen::Affine3d x_ref;
    x_ref.matrix() = x.matrix() + delta_x;
    std::cout<<"x_ref: "<<x_ref.matrix()<<std::endl;

    Eigen::MatrixXd J(6, _model_ptr->getNv());
    J.setZero();
    _model_ptr->getJacobian("r_sole",J);
    EXPECT_TRUE(cartesian.getA() == J);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == Eigen::MatrixXd::Identity(6,6));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 1.0;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(x_ref.matrix());
    cartesian.update();
    Eigen::Vector3d positionError, orientationError;
    cartesian_utils::computeCartesianError(x.matrix(), x_ref.matrix(),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    Eigen::VectorXd tmp(6);
    tmp<<cartesian.getLambda()*positionError,-cartesian.getLambda()*orientationErrorGain*orientationError;
    EXPECT_TRUE(cartesian.getb() == tmp);

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<tmp<<"]"<<std::endl;

    Eigen::Affine3d x_now;
    for(unsigned int i = 0; i < 2000; ++i)
    {
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        cartesian._update();
        q_whole = _model_ptr->sum(q_whole, cartesian.getA().transpose()*cartesian.getb());

        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        _model_ptr->getPose("r_sole", x_now);
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }

    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
        }
    }
}


TEST_F(testCartesianTask, testCartesianTaskWorldLocal_)
{
    // setting initial position with bent legs
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();

    q_whole[_model_ptr->getQIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 *_model_ptr,
                                                 "l_sole",
                                                 "r_sole");

    Eigen::MatrixXd delta_x(4,4); delta_x.setZero(4,4);
    delta_x(2,3) = -0.02;
    Eigen::Affine3d x;
    _model_ptr->getPose("l_sole","r_sole", x);

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x.matrix()<<std::endl;
    std::cout<<"delta_x: "<<delta_x<<std::endl;

    Eigen::Affine3d x_ref;
    x_ref.matrix() = x.matrix() + delta_x;
    std::cout<<"x_ref: "<<x_ref.matrix()<<std::endl;

    Eigen::MatrixXd J(6, _model_ptr->getNv());
    J.setZero();
    _model_ptr->getRelativeJacobian("l_sole","r_sole", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == Eigen::MatrixXd::Identity(6,6));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.5;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(x_ref.matrix());
    cartesian.update();
    Eigen::Vector3d positionError, orientationError;
    cartesian_utils::computeCartesianError(x.matrix(), x_ref.matrix(),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    Eigen::VectorXd tmp(6);
    tmp<<cartesian.getLambda()*positionError,-cartesian.getLambda()*orientationErrorGain*orientationError;
    EXPECT_TRUE(cartesian.getb() == tmp);


    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<tmp<<"]"<<std::endl;

    Eigen::Affine3d x_now;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        cartesian._update();
        q_whole = _model_ptr->sum(q_whole, cartesian.getA().transpose()*cartesian.getb());
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        _model_ptr->getPose("l_sole","r_sole", x_now);
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }


    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
        }
    }

}


TEST_F(testCartesianTask, testCartesianTaskRelativeUpdateWorld_)
{
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();

    q_whole[_model_ptr->getQIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LShSag")] = 20.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 *_model_ptr,
                                                 "l_wrist",
                                                 "l_sole");

    XBot::ModelInterface& reference_to_model_interface = *_model_ptr;
    Eigen::MatrixXd JJJ(6, q_whole.size());
    reference_to_model_interface.getRelativeJacobian("l_sole", "l_wrist", JJJ);
    std::cout<<"reference_to_model_interface J: "<<JJJ<<std::endl;

    Eigen::MatrixXd delta_x(4,4); delta_x.setZero(4,4);
    delta_x(2,3) = -0.02;
    Eigen::Affine3d x;
    _model_ptr->getPose("l_wrist","l_sole", x);

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x.matrix()<<std::endl;
    std::cout<<"delta_x: "<<delta_x<<std::endl;

    Eigen::Affine3d x_ref;
    x_ref.matrix() = x.matrix() + delta_x;
    std::cout<<"x_ref: "<<x_ref.matrix()<<std::endl;

    Eigen::MatrixXd J(6, _model_ptr->getNv());
    J.setZero();
    _model_ptr->getRelativeJacobian("l_wrist","l_sole", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    std::cout<<"J model: "<<J<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == Eigen::MatrixXd::Identity(6,6));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.5;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(x_ref.matrix());
    cartesian.update();
    Eigen::Vector3d positionError, orientationError;
    cartesian_utils::computeCartesianError(x.matrix(), x_ref.matrix(),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    Eigen::VectorXd tmp(6);
    tmp<<cartesian.getLambda()*positionError,-cartesian.getLambda()*orientationErrorGain*orientationError;
    EXPECT_TRUE(cartesian.getb() == tmp);


    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<tmp<<"]"<<std::endl;

    Eigen::Affine3d x_now;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();
        cartesian._update();
        q_whole = _model_ptr->sum(q_whole, cartesian.getA().transpose()*cartesian.getb());
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();
        _model_ptr->getPose("l_wrist","l_sole", x_now);
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
        }
    }
}

TEST_F(testCartesianTask, testCartesianTaskRelativeWaistUpdateWorld_)
{
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();

    q_whole[_model_ptr->getQIndex("LShSag")] = 20.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LElbj")] = -80.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 *_model_ptr,
                                                 "l_wrist",
                                                 "Waist");
    XBot::ModelInterface& reference_to_model_interface = *_model_ptr;
    Eigen::MatrixXd JJJ(6, q_whole.size());
    reference_to_model_interface.getRelativeJacobian("Waist", "l_wrist", JJJ);
    std::cout<<"reference_to_model_interface J: "<<JJJ<<std::endl;

    Eigen::MatrixXd delta_x(4,4); delta_x.setZero(4,4);
    delta_x(2,3) = -0.02;
    Eigen::Affine3d x;
    _model_ptr->getPose("l_wrist","Waist", x);

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x.matrix()<<std::endl;
    std::cout<<"delta_x: "<<delta_x<<std::endl;

    Eigen::Affine3d x_ref;
    x_ref.matrix() = x.matrix() + delta_x;
    std::cout<<"x_ref: "<<x_ref.matrix()<<std::endl;

    Eigen::MatrixXd J(6, _model_ptr->getNv());
    J.setZero();
    _model_ptr->getRelativeJacobian("l_wrist","Waist", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    std::cout<<"J model: "<<J<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == Eigen::MatrixXd::Identity(6,6));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.5;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(x_ref.matrix());
    cartesian.update();
    Eigen::Vector3d positionError, orientationError;
    cartesian_utils::computeCartesianError(x.matrix(), x_ref.matrix(),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    Eigen::VectorXd tmp(6);
    tmp<<cartesian.getLambda()*positionError,-cartesian.getLambda()*orientationErrorGain*orientationError;
    EXPECT_TRUE(cartesian.getb() == tmp);


    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<tmp<<"]"<<std::endl;

    Eigen::Affine3d x_now;
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();
        cartesian._update();
        q_whole = _model_ptr->sum(q_whole, cartesian.getA().transpose()*cartesian.getb());
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();
        _model_ptr->getPose("l_wrist","Waist", x_now);
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }


    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
        }
    }
}

TEST_F(testCartesianTask, testActiveJointsMask)
{
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();


    q_whole[_model_ptr->getQIndex("LHipSag")] = -20.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LHipLat")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LHipYaw")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LKneeSag")] = -80.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LAnkLat")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LAnkSag")] = -10.0*M_PI/180.0;

    q_whole[_model_ptr->getQIndex("WaistLat")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("WaistSag")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("WaistYaw")] = -10.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::torso",
                                                 *_model_ptr,
                                                 "torso",
                                                 "l_sole");

    cartesian.update();
    std::cout<<"J:\n "<<cartesian.getA()<<std::endl;

    std::vector<bool> active_joint_mask = cartesian.getActiveJointsMask();
    for(unsigned int i = 0; i < active_joint_mask.size(); ++i)
        EXPECT_TRUE(active_joint_mask[i]);

    Eigen::MatrixXd J = cartesian.getA();

    Eigen::MatrixXd J_torso(6,3); J_torso.setZero(6,3);
    J_torso.col(0) = J.col((int)_model_ptr->getQIndex("WaistLat"));
    J_torso.col(1) = J.col((int)_model_ptr->getQIndex("WaistSag"));
    J_torso.col(2) = J.col((int)_model_ptr->getQIndex("WaistYaw"));

    std::cout<<"J_torso:\n "<<J_torso<<std::endl;
    std::cout<<std::endl;
    EXPECT_FALSE(J_torso == Eigen::MatrixXd::Zero(J_torso.rows(), J_torso.cols()));

    Eigen::MatrixXd J_left_leg(6,6);
    J_left_leg.col(0) = J.col((int)_model_ptr->getQIndex("LHipSag"));
    J_left_leg.col(1) = J.col((int)_model_ptr->getQIndex("LHipLat"));
    J_left_leg.col(2) = J.col((int)_model_ptr->getQIndex("LHipYaw"));
    J_left_leg.col(3) = J.col((int)_model_ptr->getQIndex("LKneeSag"));
    J_left_leg.col(4) = J.col((int)_model_ptr->getQIndex("LAnkLat"));
    J_left_leg.col(5) = J.col((int)_model_ptr->getQIndex("LAnkSag"));
    std::cout<<"J_left_leg:\n "<<J_left_leg<<std::endl;
    EXPECT_FALSE(J_left_leg == Eigen::MatrixXd::Zero(J_left_leg.rows(), J_left_leg.cols()));

    active_joint_mask[(int)_model_ptr->getQIndex("LHipSag")] = false;
    active_joint_mask[(int)_model_ptr->getQIndex("LHipLat")] = false;
    active_joint_mask[(int)_model_ptr->getQIndex("LHipYaw")] = false;
    active_joint_mask[(int)_model_ptr->getQIndex("LKneeSag")] = false;
    active_joint_mask[(int)_model_ptr->getQIndex("LAnkLat")] = false;
    active_joint_mask[(int)_model_ptr->getQIndex("LAnkSag")] = false;

    cartesian.setActiveJointsMask(active_joint_mask);

    J = cartesian.getA();

    Eigen::MatrixXd J_torso2(6,3);
    J_torso2.col(0) = J.col((int)_model_ptr->getQIndex("WaistLat"));
    J_torso2.col(1) = J.col((int)_model_ptr->getQIndex("WaistSag"));
    J_torso2.col(2) = J.col((int)_model_ptr->getQIndex("WaistYaw"));
    std::cout<<"J_torso:\n "<<J_torso2<<std::endl;
    std::cout<<std::endl;
    EXPECT_FALSE(J_torso2 == Eigen::MatrixXd::Zero(J_torso.rows(), J_torso.cols()));
    EXPECT_TRUE(J_torso == J_torso2);

    J_left_leg.col(0) = J.col((int)_model_ptr->getQIndex("LHipSag"));
    J_left_leg.col(1) = J.col((int)_model_ptr->getQIndex("LHipLat"));
    J_left_leg.col(2) = J.col((int)_model_ptr->getQIndex("LHipYaw"));
    J_left_leg.col(3) = J.col((int)_model_ptr->getQIndex("LKneeSag"));
    J_left_leg.col(4) = J.col((int)_model_ptr->getQIndex("LAnkLat"));
    J_left_leg.col(5) = J.col((int)_model_ptr->getQIndex("LAnkSag"));
    std::cout<<"J_left_leg:\n "<<J_left_leg<<std::endl;
    EXPECT_TRUE(J_left_leg == Eigen::MatrixXd::Zero(J_left_leg.rows(), J_left_leg.cols()));


    std::cout<<"J:\n "<<cartesian.getA()<<std::endl;
}

TEST_F(testCartesianTask, testReset)
{
    Eigen::VectorXd q_whole = _model_ptr->getNeutralQ();


    q_whole[_model_ptr->getQIndex("LHipSag")] = -20.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LHipLat")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LHipYaw")] = 10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LKneeSag")] = -80.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LAnkLat")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("LAnkSag")] = -10.0*M_PI/180.0;

    q_whole[_model_ptr->getQIndex("WaistLat")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("WaistSag")] = -10.0*M_PI/180.0;
    q_whole[_model_ptr->getQIndex("WaistYaw")] = -10.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::torso",
                                                 *_model_ptr,
                                                 "torso",
                                                 "l_sole");

    std::cout<<"INITIALIZATION: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    Eigen::MatrixXd actual_pose = cartesian.getActualPose();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    Eigen::MatrixXd reference_pose = cartesian.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<cartesian.getb()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_EQ(actual_pose(i,j), reference_pose(i,j));
    }


    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_EQ(cartesian.getb()[i], 0);


    std::cout<<"CHANGING q: ACTUAL AND REFERENCE DIFFERENT, b IS NOT 0"<<std::endl;
    q_whole = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    cartesian.update();

    actual_pose = cartesian.getActualPose();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    reference_pose = cartesian.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<cartesian.getb()<<std::endl;

    std::cout<<"RESET: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    EXPECT_TRUE(cartesian.reset());
    actual_pose = cartesian.getActualPose();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    reference_pose = cartesian.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<cartesian.getb()<<std::endl;

    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j < 4; ++j)
            EXPECT_EQ(actual_pose(i,j), reference_pose(i,j));
    }


    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_EQ(cartesian.getb()[i], 0);


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
