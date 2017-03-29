#include <idynutils/tests_utils.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <gtest/gtest.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/stacks/velocity/ManipulationStack.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>

using namespace yarp::math;
using namespace OpenSoT;

namespace {

class testGazeTask: public ::testing::Test
{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
protected:
    iDynUtils _robot;
    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string _path_to_cfg;

    testGazeTask() : _robot("bigman",
                             std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                             std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf")
    {
        std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
        std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

        _path_to_cfg = robotology_root + relative_path;

        _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
                (XBot::ModelInterface::getModel(_path_to_cfg));
        _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&_robot, &null_deleter));

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    }

    virtual ~testGazeTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};


TEST_F(testGazeTask, testGazeTaskWorldGlobal_)
{
    // setting initial position with bent legs
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);

    OpenSoT::DefaultHumanoidStack DHS(_robot, 0.001, conversion_utils_YARP::toEigen(q_whole));

    DHS.velocityLimits->setVelocityLimits(0.6);

    EXPECT_EQ(DHS.gaze->getA().rows(), 2);
    EXPECT_EQ(DHS.gaze->getb().size(), 2);

    EXPECT_TRUE(DHS.gaze->getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(2,2).eye()));

    EXPECT_TRUE(DHS.gaze->getConstraints().size() == 0);

    std::vector<bool> gaze_active_joint_mask = DHS.gaze->getActiveJointsMask();
    for(unsigned int i = 0; i < gaze_active_joint_mask.size(); ++i)
        gaze_active_joint_mask[i] = false;
    for(unsigned int i = 0; i < _robot.head.joint_numbers.size(); ++i)
        gaze_active_joint_mask[_robot.head.joint_numbers[i]] = true;
    DHS.gaze->setActiveJointsMask(gaze_active_joint_mask);

    gaze_active_joint_mask = DHS.gaze->getActiveJointsMask();
    for(unsigned int i = 0; i < gaze_active_joint_mask.size(); ++i)
        if(std::find(_robot.head.joint_numbers.begin(),
                     _robot.head.joint_numbers.end(),
                     i) != _robot.head.joint_numbers.end())
            EXPECT_TRUE(gaze_active_joint_mask[i]);
        else
            EXPECT_FALSE(gaze_active_joint_mask[i]);

    double K = 0.1;
    DHS.gaze->setLambda(K);
    EXPECT_DOUBLE_EQ(DHS.gaze->getLambda(), K);

    OpenSoT::AutoStack::Ptr stack = (DHS.gaze / DHS.postural)
                                    << DHS.velocityLimits
                                    << DHS.jointLimits;

    //DHS.gaze->setGaze(x_ref);
    stack->update(conversion_utils_YARP::toEigen(q_whole));
    yarp::sig::Matrix J_1 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getA());
    yarp::sig::Vector b_1 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getb());

    q_whole[_robot.right_arm.joint_numbers[0]] += 0.1;
    _robot.updateiDyn3Model(q_whole, true);
    stack->update(conversion_utils_YARP::toEigen(q_whole));
    yarp::sig::Matrix J_2 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getA());
    yarp::sig::Vector b_2 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getb());

    q_whole[_robot.head.joint_numbers[0]] += 0.1;
    q_whole[_robot.head.joint_numbers[1]] += 0.1;
    _robot.updateiDyn3Model(q_whole, true);
    stack->update(conversion_utils_YARP::toEigen(q_whole));
    yarp::sig::Matrix J_3 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getA());
    yarp::sig::Vector b_3 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getb());

    q_whole[_robot.torso.joint_numbers[0]] += 0.1;
    q_whole[_robot.torso.joint_numbers[1]] += 0.1;
    q_whole[_robot.torso.joint_numbers[2]] += 0.1;
    _robot.updateiDyn3Model(q_whole, true);
    stack->update(conversion_utils_YARP::toEigen(q_whole));
    yarp::sig::Matrix J_4 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getA());
    yarp::sig::Vector b_4 = conversion_utils_YARP::fromEigentoYarp(DHS.gaze->getb());

    EXPECT_TRUE(J_1 == J_2);
    EXPECT_TRUE(b_1 == b_2);
    EXPECT_FALSE(J_1 == J_3);
    EXPECT_FALSE(b_1 == b_3);
    EXPECT_FALSE(J_1 == J_4);
    EXPECT_FALSE(b_1 == b_4);

/*
    double orientationErrorGain = 0.5;
    DHS.gaze->setOrientationErrorGain(orientationErrorGain);
    EXPECT_EQ(DHS.gaze->getOrientationErrorGain(), 0.5);

    EXPECT_EQ(Gaze.getb(), yarp::sig::Vector(2, 0.0));

    yarp::sig::Matrix x_now;
    for(unsigned int i = 0; i < 60; ++i)
    {
        _robot.updateiDyn3Model(q_whole,true);
        Gaze._update(q_whole);
        q_whole += pinv(Gaze.getA(),1E-7)*Gaze.getb();
        _robot.updateiDyn3Model(q_whole,true);
        x_now = _robot.iDyn3_model.getPosition(
                      _robot.right_leg.end_effector_index);
        //std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    EXPECT_LT( findMax((x_ref-x_now).subcol(0,3,3)), 1E-3 );
    EXPECT_LT( abs(findMin((x_ref-x_now).subcol(0,3,3))), 1E-3 );
    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
        }
    }

    for(unsigned int i = 0; i < 6; ++i)
        EXPECT_NEAR(q_whole[_robot.left_leg.joint_numbers[i]],0.0,1E-16);
    for(unsigned int i = 0; i < 7; ++i) {
        EXPECT_DOUBLE_EQ(q_whole[_robot.left_arm.joint_numbers[i]],0.0);
        EXPECT_DOUBLE_EQ(q_whole[_robot.right_arm.joint_numbers[i]],0.0);
    }
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(q_whole[_robot.torso.joint_numbers[i]],0.0);
    }
    */
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
