#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <boost/make_shared.hpp>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>


using namespace yarp::math;

namespace {

class testCartesianTask: public ::testing::Test
{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
protected:
    iDynUtils _robot;
    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string _path_to_cfg;

    testCartesianTask() : _robot("coman",
                                 std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                                 std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf")
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

    virtual ~testCartesianTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};


TEST_F(testCartesianTask, testCartesianTaskWorldGlobal_)
{
    // setting initial position with bent legs
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);

    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::right_leg",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "r_sole",
                                                 "world");

    // setting x_ref with a delta offset along the z axis (-2cm)
    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
                      delta_x(2,3) = -0.02;
    Eigen::MatrixXd x = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("r_sole"));

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"delta_x: "<<delta_x.toString()<<std::endl;

    yarp::sig::Matrix x_ref = conversion_utils_YARP::toYARP(x) + delta_x;
    std::cout<<"x_ref: "<<x_ref.toString()<<std::endl;

    KDL::Jacobian J; J.resize(q_whole.size());
    _model_ptr->getJacobian("r_sole",KDL::Vector::Zero(), J);
    EXPECT_TRUE(cartesian.getA() == J.data);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.1;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
    Eigen::VectorXd positionError, orientationError;
    cartesian_utils::computeCartesianError(x, conversion_utils_YARP::toEigen(x_ref),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    EXPECT_TRUE(cartesian.getb() == conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                        conversion_utils_YARP::toYARP(positionError),
                                        -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError))));

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                              conversion_utils_YARP::toYARP(positionError),
                                                              -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)))<<"]"<<std::endl;

    Eigen::MatrixXd x_now;
    for(unsigned int i = 0; i < 60; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        x_now = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("r_sole"));
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    EXPECT_LT( findMax((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3)), 1E-3 );
    EXPECT_LT( abs(findMin((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3))), 1E-3 );
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
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _robot.switchAnchorAndFloatingBase("r_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "l_sole",
                                                 "world");

    // setting x_ref with a delta offset along the z axis (-2cm)
    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
                      delta_x(2,3) = -0.02;
    Eigen::MatrixXd x = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("l_sole"));

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"delta_x: "<<delta_x.toString()<<std::endl;

    yarp::sig::Matrix x_ref = conversion_utils_YARP::toYARP(x) + delta_x;
    std::cout<<"x_ref: "<<x_ref.toString()<<std::endl;

    KDL::Jacobian J; J.resize(q_whole.size());
    _model_ptr->getJacobian("l_sole",KDL::Vector::Zero(), J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J.data);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.1;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
    Eigen::VectorXd positionError, orientationError;
    cartesian_utils::computeCartesianError(x, conversion_utils_YARP::toEigen(x_ref),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    EXPECT_TRUE(cartesian.getb() == conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                        conversion_utils_YARP::toYARP(positionError),
                                        -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError))));

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                              conversion_utils_YARP::toYARP(positionError),
                                                              -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)))<<"]"<<std::endl;

    Eigen::MatrixXd x_now;
    for(unsigned int i = 0; i < 60; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        x_now = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("l_sole"));
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    EXPECT_LT( findMax((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3)), 1E-3 );
    EXPECT_LT( abs(findMin((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3))), 1E-3 );
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


TEST_F(testCartesianTask, testCartesianTaskRelativeNoUpdateWorld_)
{
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    _robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), false);


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "l_wrist",
                                                 "l_sole");
    XBot::ModelInterface& reference_to_model_interface = *_model_ptr;
    Eigen::MatrixXd JJJ(6, q_whole.size());
    reference_to_model_interface.getRelativeJacobian("l_sole", "l_wrist", JJJ);
    std::cout<<"reference_to_model_interface J: "<<JJJ<<std::endl;

    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
                      delta_x(2,3) = -0.02;
    Eigen::MatrixXd x = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("l_sole"),
                                           _robot.iDynTree_model.getLinkIndex("l_wrist"));

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"delta_x: "<<delta_x.toString()<<std::endl;

    yarp::sig::Matrix x_ref = conversion_utils_YARP::toYARP(x) + delta_x;
    std::cout<<"x_ref: "<<x_ref.toString()<<std::endl;

    KDL::Jacobian J; J.resize(q_whole.size());
    _model_ptr->getRelativeJacobian("l_sole","l_wrist", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    std::cout<<"J model: "<<J.data<<std::endl;
    Eigen::MatrixXd JJ(6, q_whole.size());
    _robot.getRelativeJacobian(_robot.iDynTree_model.getLinkIndex("l_wrist"),
                               _robot.iDynTree_model.getLinkIndex("l_sole"), JJ);
    std::cout<<"J robot: "<<JJ<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J.data);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.1;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
    Eigen::VectorXd positionError, orientationError;
    cartesian_utils::computeCartesianError(x, conversion_utils_YARP::toEigen(x_ref),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    EXPECT_TRUE(cartesian.getb() == conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                        conversion_utils_YARP::toYARP(positionError),
                                        -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError))));

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                              conversion_utils_YARP::toYARP(positionError),
                                                              -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)))<<"]"<<std::endl;

    Eigen::MatrixXd x_now;
    for(unsigned int i = 0; i < 60; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),false);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),false);
        x_now = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("l_sole"),
                                   _robot.iDynTree_model.getLinkIndex("l_wrist"));
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    EXPECT_LT( findMax((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3)), 1E-3 );
    EXPECT_LT( abs(findMin((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3))), 1E-3 );
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

TEST_F(testCartesianTask, testCartesianTaskRelativeWaistNoUpdateWorld_)
{
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    _robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), false);


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "l_wrist",
                                                 "Waist");
    XBot::ModelInterface& reference_to_model_interface = *_model_ptr;
    Eigen::MatrixXd JJJ(6, q_whole.size());
    reference_to_model_interface.getRelativeJacobian("Waist", "l_wrist", JJJ);
    std::cout<<"reference_to_model_interface J: "<<JJJ<<std::endl;

    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
                      delta_x(2,3) = -0.02;
    Eigen::MatrixXd x = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("Waist"),
                                           _robot.iDynTree_model.getLinkIndex("l_wrist"));

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"delta_x: "<<delta_x.toString()<<std::endl;

    yarp::sig::Matrix x_ref = conversion_utils_YARP::toYARP(x) + delta_x;
    std::cout<<"x_ref: "<<x_ref.toString()<<std::endl;

    KDL::Jacobian J; J.resize(q_whole.size());
    _model_ptr->getRelativeJacobian("Waist","l_wrist", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    std::cout<<"J model: "<<J.data<<std::endl;
    Eigen::MatrixXd JJ(6, q_whole.size());
    _robot.getRelativeJacobian(_robot.iDynTree_model.getLinkIndex("l_wrist"),
                               _robot.iDynTree_model.getLinkIndex("Waist"), JJ);
    std::cout<<"J robot: "<<JJ<<std::endl;
    EXPECT_TRUE(cartesian.getA() == J.data);
    EXPECT_EQ(cartesian.getA().rows(), 6);
    EXPECT_EQ(cartesian.getb().size(), 6);

    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

    double K = 0.1;
    cartesian.setLambda(K);
    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
    Eigen::VectorXd positionError, orientationError;
    cartesian_utils::computeCartesianError(x, conversion_utils_YARP::toEigen(x_ref),
                                           positionError, orientationError);

    double orientationErrorGain = 1.0;
    cartesian.setOrientationErrorGain(orientationErrorGain);

    EXPECT_TRUE(cartesian.getb() == conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                        conversion_utils_YARP::toYARP(positionError),
                                        -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError))));

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                              conversion_utils_YARP::toYARP(positionError),
                                                              -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)))<<"]"<<std::endl;

    Eigen::MatrixXd x_now;
    for(unsigned int i = 0; i < 60; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),false);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),false);
        x_now = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("Waist"),
                                   _robot.iDynTree_model.getLinkIndex("l_wrist"));
        std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
    }



    EXPECT_LT( findMax((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3)), 1E-3 );
    EXPECT_LT( abs(findMin((x_ref-conversion_utils_YARP::toYARP(x_now)).subcol(0,3,3))), 1E-3 );
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


//TEST_F(testCartesianTask, testCartesianTaskRelativeUpdateWorld_)
//{
//    bool update_world = true;
//    // setting initial position with bent legs
//    yarp::sig::Vector q_leg(6, 0.0),
//                      q_whole(_robot.iDyn3_model.getNrOfDOFs(), 0.0);
//    q_leg[0] = -25.0*M_PI/180.0;
//    q_leg[3] =  50.0*M_PI/180.0;
//    q_leg[5] = -25.0*M_PI/180.0;

//    _robot.fromRobotToIDyn(q_leg, q_whole, _robot.left_leg);

//    yarp::sig::Vector arm(_robot.left_arm.getNrOfDOFs(), 0.0);
//    arm[0] = 20.0 * M_PI/180.0;
//    arm[1] = 10.0 * M_PI/180.0;
//    arm[3] = -80.0 * M_PI/180.0;
//    _robot.fromRobotToIDyn(arm, q_whole, _robot.left_arm);

//    _robot.updateiDyn3Model(q_whole, update_world);

//    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
//                                                 conversion_utils_YARP::toEigen(q_whole),
//                                                 *(_model_ptr.get()),
//                                                 "l_wrist",
//                                                 "l_sole");


//    // setting x_ref with a delta offset along the z axis (-2cm)
//    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
//                      delta_x(2,3) = -0.02;
//    yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(
//                            _robot.left_leg.end_effector_index,
//                            _robot.left_arm.end_effector_index);
//    yarp::sig::Matrix x_ref = x + delta_x;

//    yarp::sig::Matrix J;
//    _robot.iDyn3_model.getRelativeJacobian(_robot.left_arm.end_effector_index,
//                                           _robot.left_leg.end_effector_index, J);
//    //EXPECT_TRUE(cartesian.getA() == J);
//    EXPECT_EQ(cartesian.getA().rows(), 6);
//    EXPECT_EQ(cartesian.getb().size(), 6);

//    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

//    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

//    double K = 0.1;
//    cartesian.setLambda(K);
//    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

//    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
//    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
//    yarp::sig::Vector positionError, orientationError;
//    cartesian_utils::computeCartesianError(x, x_ref,
//                                           positionError, orientationError);

//    double orientationErrorGain = 1.0;
//    cartesian.setOrientationErrorGain(orientationErrorGain);

//    EXPECT_TRUE(cartesian_utils::fromEigentoYarp(cartesian.getb()) == cartesian.getLambda()*cat(positionError, -orientationErrorGain*orientationError));

//    yarp::sig::Matrix x_now;
//    for(unsigned int i = 0; i < 120; ++i)
//    {
//        _robot.updateiDyn3Model(q_whole, update_world);
//        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
//        q_whole += pinv(cartesian_utils::fromEigentoYarp(cartesian.getA()),1E-7)*
//                cartesian_utils::fromEigentoYarp(cartesian.getb());
//        _robot.updateiDyn3Model(q_whole, update_world);
//        x_now = _robot.iDyn3_model.getPosition(
//                      _robot.left_leg.end_effector_index,
//                      _robot.left_arm.end_effector_index);
//        //std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
//    }



//    EXPECT_LT( findMax((x_ref-x_now).subcol(0,3,3)), 1E-3 );
//    EXPECT_LT( abs(findMin((x_ref-x_now).subcol(0,3,3))), 1E-3 );
//    // checking for the position
//    for(unsigned int i = 0; i < 3; ++i) {
//        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
//    }
//    for(unsigned int i = 0; i < 3; ++i) {
//        for(unsigned int j = 0; j < 3; ++j) {
//            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
//        }
//    }

//    for(unsigned int i = 0; i < 6; ++i)
//        EXPECT_DOUBLE_EQ(q_whole[_robot.right_leg.joint_numbers[i]],0.0);
//    for(unsigned int i = 0; i < 7; ++i) {
//        EXPECT_DOUBLE_EQ(q_whole[_robot.right_arm.joint_numbers[i]],0.0);
//    }
//}

//TEST_F(testCartesianTask, testCartesianTaskRelativeWaistUpdateWorld_)
//{
//    bool update_world = true;
//    // setting initial position with bent legs
//    yarp::sig::Vector q_whole(_robot.iDyn3_model.getNrOfDOFs(), 0.0);

//    yarp::sig::Vector arm(_robot.left_arm.getNrOfDOFs(), 0.0);
//    arm[0] = 20.0 * M_PI/180.0;
//    arm[1] = 10.0 * M_PI/180.0;
//    arm[3] = -80.0 * M_PI/180.0;
//    _robot.fromRobotToIDyn(arm, q_whole, _robot.left_arm);

//    _robot.updateiDyn3Model(q_whole, update_world);

//    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::l_wrist",
//                                                 conversion_utils_YARP::toEigen(q_whole),
//                                                 *(_model_ptr.get()),
//                                                 "l_wrist",
//                                                 "Waist");

//    // setting x_ref with a delta offset along the z axis (-2cm)
//    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
//                      delta_x(2,3) = -0.02;
//    yarp::sig::Matrix x = _robot.iDyn3_model.getPosition(
//                            0,
//                            _robot.left_arm.end_effector_index);
//    yarp::sig::Matrix x_ref = x + delta_x;

//    yarp::sig::Matrix J;
//    _robot.iDyn3_model.getRelativeJacobian(_robot.left_arm.end_effector_index,
//                                           0, J);
//    //EXPECT_TRUE(cartesian.getA() == J);
//    EXPECT_EQ(cartesian.getA().rows(), 6);
//    EXPECT_EQ(cartesian.getb().size(), 6);

//    EXPECT_TRUE(cartesian.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(6,6).eye()));

//    EXPECT_TRUE(cartesian.getConstraints().size() == 0);

//    double K = 0.1;
//    cartesian.setLambda(K);
//    EXPECT_DOUBLE_EQ(cartesian.getLambda(), K);

//    cartesian.setReference(conversion_utils_YARP::toEigen(x_ref));
//    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
//    yarp::sig::Vector positionError, orientationError;
//    cartesian_utils::computeCartesianError(x, x_ref,
//                                           positionError, orientationError);

//    double orientationErrorGain = 1.0;
//    cartesian.setOrientationErrorGain(orientationErrorGain);

//    EXPECT_TRUE(cartesian_utils::fromEigentoYarp(cartesian.getb()) == cartesian.getLambda()*cat(positionError, -orientationErrorGain*orientationError));

//    yarp::sig::Matrix x_now;
//    for(unsigned int i = 0; i < 120; ++i)
//    {
//        _robot.updateiDyn3Model(q_whole, update_world);
//        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
//        q_whole += pinv(cartesian_utils::fromEigentoYarp(cartesian.getA()),1E-7)*
//                cartesian_utils::fromEigentoYarp(cartesian.getb());
//        _robot.updateiDyn3Model(q_whole, update_world);
//        x_now = _robot.iDyn3_model.getPosition(
//                      0,
//                      _robot.left_arm.end_effector_index);
//        //std::cout << "Current error at iteration " << i << " is " << x_ref(2,3) - x_now(2,3) << std::endl;
//    }



//    EXPECT_LT( findMax((x_ref-x_now).subcol(0,3,3)), 1E-3 );
//    EXPECT_LT( abs(findMin((x_ref-x_now).subcol(0,3,3))), 1E-3 );
//    // checking for the position
//    for(unsigned int i = 0; i < 3; ++i) {
//        EXPECT_NEAR(x_ref(i,3),x_now(i,3),1E-4);
//    }
//    for(unsigned int i = 0; i < 3; ++i) {
//        for(unsigned int j = 0; j < 3; ++j) {
//            EXPECT_NEAR(x_ref(i,j),x_now(i,j),1E-4);
//        }
//    }

//    for(unsigned int i = 0; i < 6; ++i) {
//        EXPECT_NEAR(q_whole[_robot.left_leg.joint_numbers[i]],0.0,1E-16);
//        EXPECT_NEAR(q_whole[_robot.right_leg.joint_numbers[i]],0.0,1E-16);
//    }
//    for(unsigned int i = 0; i < 7; ++i) {
//        EXPECT_DOUBLE_EQ(q_whole[_robot.right_arm.joint_numbers[i]],0.0);
//    }

//}

//TEST_F(testCartesianTask, testActiveJointsMask)
//{
//    bool update_world = true;
//    // setting initial position with bent legs
//    yarp::sig::Vector q_whole(_robot.iDyn3_model.getNrOfDOFs(), 0.0);

//    _robot.setFloatingBaseLink(_robot.left_leg.end_effector_name);

//    yarp::sig::Vector l_leg(_robot.left_leg.getNrOfDOFs(), 0.0);
//    l_leg[0] = -20.0 * M_PI/180.0;
//    l_leg[1] = 10.0 * M_PI/180.0;
//    l_leg[2] = 10.0 * M_PI/180.0;
//    l_leg[3] = -80.0 * M_PI/180.0;
//    l_leg[4] = -10.0 * M_PI/180.0;
//    l_leg[5] = -10.0 * M_PI/180.0;
//    _robot.fromRobotToIDyn(l_leg, q_whole, _robot.left_leg);
//    yarp::sig::Vector torso(_robot.torso.getNrOfDOFs(), 0.0);
//    torso[0] = -10.0 * M_PI/180.0;
//    torso[1] = -10.0 * M_PI/180.0;
//    torso[2] = -10.0 * M_PI/180.0;
//    _robot.fromRobotToIDyn(torso, q_whole, _robot.torso);

//    _robot.updateiDyn3Model(q_whole, update_world);

//    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::torso",
//                                                 conversion_utils_YARP::toEigen(q_whole),
//                                                 *(_model_ptr.get()),
//                                                 "torso",
//                                                 "world");

//    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
//    std::vector<bool> active_joint_mask = cartesian.getActiveJointsMask();
//    for(unsigned int i = 0; i < active_joint_mask.size(); ++i)
//        EXPECT_TRUE(active_joint_mask[i]);

//    yarp::sig::Matrix J = cartesian_utils::fromEigentoYarp(cartesian.getA());

//    yarp::sig::Matrix J_torso(6,_robot.torso.getNrOfDOFs());
//    for(unsigned int i = 0; i < _robot.torso.getNrOfDOFs(); ++i)
//        J_torso.setCol(i, J.getCol((int)_robot.torso.joint_numbers[i]));
//    //std::cout<<"J_torso:\n "<<J_torso.toString()<<std::cout;
//    std::cout<<std::endl;
//    EXPECT_FALSE(J_torso == yarp::sig::Matrix(J_torso.rows(), J_torso.cols()));

//    yarp::sig::Matrix J_left_leg(6,_robot.left_leg.getNrOfDOFs());
//    for(unsigned int i = 0; i < _robot.left_leg.getNrOfDOFs(); ++i)
//        J_left_leg.setCol(i, J.getCol((int)_robot.left_leg.joint_numbers[i]));
//    //std::cout<<"J_left_leg:\n "<<J_left_leg.toString()<<std::cout;
//    EXPECT_FALSE(J_left_leg == yarp::sig::Matrix(J_left_leg.rows(), J_left_leg.cols()));

//    for(unsigned int i = 0; i < _robot.left_leg.getNrOfDOFs(); ++i)
//        active_joint_mask[_robot.left_leg.joint_numbers[i]] = false;

//    cartesian.setActiveJointsMask(active_joint_mask);

//    J = cartesian_utils::fromEigentoYarp(cartesian.getA());

//    yarp::sig::Matrix J_torso2(6,_robot.torso.getNrOfDOFs());
//    for(unsigned int i = 0; i < _robot.torso.getNrOfDOFs(); ++i)
//        J_torso2.setCol(i, J.getCol((int)_robot.torso.joint_numbers[i]));
//    //std::cout<<"J_torso:\n "<<J_torso2.toString()<<std::cout;
//    std::cout<<std::endl;
//    EXPECT_FALSE(J_torso2 == yarp::sig::Matrix(J_torso.rows(), J_torso.cols()));
//    EXPECT_TRUE(J_torso == J_torso2);

//    for(unsigned int i = 0; i < _robot.left_leg.getNrOfDOFs(); ++i)
//        J_left_leg.setCol(i, J.getCol((int)_robot.left_leg.joint_numbers[i]));
//    //std::cout<<"J_left_leg:\n "<<J_left_leg.toString()<<std::cout;
//    EXPECT_TRUE(J_left_leg == yarp::sig::Matrix(J_left_leg.rows(), J_left_leg.cols()));



//}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
