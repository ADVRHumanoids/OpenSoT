#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <boost/make_shared.hpp>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <OpenSoT/utils/cartesian_utils.h>


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

TEST_F(testCartesianTask, testSetBaseLink)
{
    Eigen::VectorXd q(_model_ptr->getJointNum());
    q.setZero(q.size());

    q[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    _robot.updateiDynTreeModel(q, true);

    std::string distal_link = "l_wrist";
    std::string base_link1 = "world";
    std::string base_link2 = "l_sole";
    std::string base_link3 = "Waist";

    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm1(
        new OpenSoT::tasks::velocity::Cartesian("l_arm",q,*(_model_ptr.get()),distal_link,
                                                base_link1));
    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm2(
        new OpenSoT::tasks::velocity::Cartesian("l_arm",q,*(_model_ptr.get()),distal_link,
                                                base_link2));
    OpenSoT::tasks::velocity::Cartesian::Ptr l_arm3(
        new OpenSoT::tasks::velocity::Cartesian("l_arm",q,*(_model_ptr.get()),distal_link,
                                                base_link3));

    l_arm1->update(q);
    l_arm2->update(q);
    l_arm3->update(q);

    EXPECT_TRUE(l_arm1->setBaseLink(base_link2));
    l_arm1->update(q);

    EXPECT_TRUE(l_arm1->getA() == l_arm2->getA());
    EXPECT_TRUE(l_arm1->getb() == l_arm2->getb());
    EXPECT_TRUE(l_arm1->getReference() == l_arm2->getReference());
    EXPECT_TRUE(l_arm1->getActualPose() == l_arm2->getActualPose());

    EXPECT_TRUE(l_arm1->setBaseLink(base_link3));
    l_arm1->update(q);

    EXPECT_TRUE(l_arm1->getA() == l_arm3->getA());
    EXPECT_TRUE(l_arm1->getb() == l_arm3->getb());
    EXPECT_TRUE(l_arm1->getReference() == l_arm3->getReference());
    EXPECT_TRUE(l_arm1->getActualPose() == l_arm3->getActualPose());
}


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

    //_robot.switchAnchorAndFloatingBase("r_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);


//    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
//                                                 conversion_utils_YARP::toEigen(q_whole),
//                                                 *(_model_ptr.get()),
//                                                 "l_sole",
//                                                 "world");
    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::left_leg",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "l_sole",
                                                 "r_sole");

    // setting x_ref with a delta offset along the z axis (-2cm)
    yarp::sig::Matrix delta_x(4,4); delta_x.zero();
                      delta_x(2,3) = -0.02;
    Eigen::MatrixXd x = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("r_sole"),
                                           _robot.iDynTree_model.getLinkIndex("l_sole"));

    std::cout<<"cartesian actual pose: "<<cartesian.getActualPose()<<std::endl;
    std::cout<<"x: "<<x<<std::endl;
    std::cout<<"delta_x: "<<delta_x.toString()<<std::endl;

    yarp::sig::Matrix x_ref = conversion_utils_YARP::toYARP(x) + delta_x;
    std::cout<<"x_ref: "<<x_ref.toString()<<std::endl;

    KDL::Jacobian J; J.resize(q_whole.size());
    _model_ptr->getRelativeJacobian("l_sole","r_sole", J);
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

    Eigen::VectorXd tmp = conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                             conversion_utils_YARP::toYARP(positionError),
                                                             -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)));

    EXPECT_NEAR((cartesian.getb() - tmp).norm(), 0.0,1e-9);

//    EXPECT_TRUE(cartesian.getb() == conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
//                                        conversion_utils_YARP::toYARP(positionError),
//                                        -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError))));

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
        x_now = _robot.getPosition(_robot.iDynTree_model.getLinkIndex("r_sole"),
                                   _robot.iDynTree_model.getLinkIndex("l_sole"));
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

    //_robot.switchAnchorAndFloatingBase("l_sole");
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
    _model_ptr->getRelativeJacobian("l_wrist","l_sole", J);
    std::cout<<"getA(): "<<cartesian.getA()<<std::endl;
    std::cout<<"J model: "<<J.data<<std::endl;
    Eigen::MatrixXd JJ(6, q_whole.size());
    _robot.getRelativeJacobian(_robot.iDynTree_model.getLinkIndex("l_wrist"),
                               _robot.iDynTree_model.getLinkIndex("l_sole"), JJ);

    KDL::Frame _tmp_kdl_frame = _robot.iDynTree_model.getPositionKDL(
                _robot.iDynTree_model.getLinkIndex("l_sole"),
                _robot.iDynTree_model.getLinkIndex("l_wrist"));

    KDL::Jacobian JJ_KDL;
    JJ_KDL.data = JJ;
    JJ_KDL.changeBase(_tmp_kdl_frame.M);

    std::cout<<"J robot: "<<JJ_KDL.data<<std::endl;
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

    //_robot.switchAnchorAndFloatingBase("l_sole");
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
    _model_ptr->getRelativeJacobian("l_wrist","Waist", J);
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


TEST_F(testCartesianTask, testCartesianTaskRelativeUpdateWorld_)
{
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    //_robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);


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
    _model_ptr->getRelativeJacobian("l_wrist","l_sole", J);
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
    for(unsigned int i = 0; i < 120; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
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

TEST_F(testCartesianTask, testCartesianTaskRelativeWaistUpdateWorld_)
{
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);

    q_whole[_robot.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    //_robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);


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
    _model_ptr->getRelativeJacobian("l_wrist","Waist", J);
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

    Eigen::VectorXd tmp = conversion_utils_YARP::toEigen(cartesian.getLambda()*cat(
                                                             conversion_utils_YARP::toYARP(positionError),
                                                             -orientationErrorGain*conversion_utils_YARP::toYARP(orientationError)));

    EXPECT_NEAR((cartesian.getb() - tmp).norm(), 0.0, 1e-9 );

    std::cout<<"cartesian.getb(): ["<<cartesian.getb()<<"]"<<std::endl;
    std::cout<<"error: ["<<tmp<<"]"<<std::endl;

    Eigen::MatrixXd x_now;
    for(unsigned int i = 0; i < 120; ++i)
    {
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
        cartesian._update(conversion_utils_YARP::toEigen(q_whole));
        q_whole += pinv(conversion_utils_YARP::toYARP(cartesian.getA()),1E-7)*
                conversion_utils_YARP::toYARP(cartesian.getb());
        _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole),true);
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

TEST_F(testCartesianTask, testActiveJointsMask)
{
    // setting initial position with bent legs
    yarp::sig::Vector q_whole(_robot.iDynTree_model.getNrOfDOFs(), 0.0);
    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -20.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LHipLat")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LHipYaw")] = 10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = -80.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkLat")] = -10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -10.0*M_PI/180.0;

    q_whole[_robot.iDynTree_model.getDOFIndex("WaistLat")] = -10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("WaistSag")] = -10.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("WaistYaw")] = -10.0*M_PI/180.0;

    //_robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_whole), true);


    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::torso",
                                                 conversion_utils_YARP::toEigen(q_whole),
                                                 *(_model_ptr.get()),
                                                 "torso",
                                                 "l_sole");
//    OpenSoT::tasks::velocity::Cartesian cartesian("cartesian::torso",
//                                                 conversion_utils_YARP::toEigen(q_whole),
//                                                 *(_model_ptr.get()),
//                                                 "torso",
//                                                 "world");

    cartesian.update(conversion_utils_YARP::toEigen(q_whole));
    std::vector<bool> active_joint_mask = cartesian.getActiveJointsMask();
    for(unsigned int i = 0; i < active_joint_mask.size(); ++i)
        EXPECT_TRUE(active_joint_mask[i]);

    yarp::sig::Matrix J = conversion_utils_YARP::toYARP(cartesian.getA());

    yarp::sig::Matrix J_torso(6,3);
    J_torso.setCol(0, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistLat")));
    J_torso.setCol(1, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistSag")));
    J_torso.setCol(2, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistYaw")));
    //std::cout<<"J_torso:\n "<<J_torso.toString()<<std::cout;
    std::cout<<std::endl;
    EXPECT_FALSE(J_torso == yarp::sig::Matrix(J_torso.rows(), J_torso.cols()));

    yarp::sig::Matrix J_left_leg(6,6);
    J_left_leg.setCol(0, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipSag")));
    J_left_leg.setCol(1, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipLat")));
    J_left_leg.setCol(2, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipYaw")));
    J_left_leg.setCol(3, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LKneeSag")));
    J_left_leg.setCol(4, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LAnkLat")));
    J_left_leg.setCol(5, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LAnkSag")));
    //std::cout<<"J_left_leg:\n "<<J_left_leg.toString()<<std::cout;
    EXPECT_FALSE(J_left_leg == yarp::sig::Matrix(J_left_leg.rows(), J_left_leg.cols()));

    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LHipSag")] = false;
    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LHipLat")] = false;
    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LHipYaw")] = false;
    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LKneeSag")] = false;
    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LAnkLat")] = false;
    active_joint_mask[(int)_robot.iDynTree_model.getDOFIndex("LAnkSag")] = false;

    cartesian.setActiveJointsMask(active_joint_mask);

    J = conversion_utils_YARP::toYARP(cartesian.getA());

    yarp::sig::Matrix J_torso2(6,3);
    J_torso2.setCol(0, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistLat")));
    J_torso2.setCol(1, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistSag")));
    J_torso2.setCol(2, J.getCol((int)_robot.iDynTree_model.getDOFIndex("WaistYaw")));
    //std::cout<<"J_torso:\n "<<J_torso2.toString()<<std::cout;
    std::cout<<std::endl;
    EXPECT_FALSE(J_torso2 == yarp::sig::Matrix(J_torso.rows(), J_torso.cols()));
    EXPECT_TRUE(J_torso == J_torso2);

    J_left_leg.setCol(0, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipSag")));
    J_left_leg.setCol(1, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipLat")));
    J_left_leg.setCol(2, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LHipYaw")));
    J_left_leg.setCol(3, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LKneeSag")));
    J_left_leg.setCol(4, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LAnkLat")));
    J_left_leg.setCol(5, J.getCol((int)_robot.iDynTree_model.getDOFIndex("LAnkSag")));
    //std::cout<<"J_left_leg:\n "<<J_left_leg.toString()<<std::cout;
    EXPECT_TRUE(J_left_leg == yarp::sig::Matrix(J_left_leg.rows(), J_left_leg.cols()));



}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
