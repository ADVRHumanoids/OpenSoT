#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <OpenSoT/utils/cartesian_utils.h>

using namespace yarp::math;

namespace {

class testCoMTask: public ::testing::Test
{
public:
    typedef idynutils2 iDynUtils;
    static void null_deleter(iDynUtils *) {}
protected:
    iDynUtils _robot;
    iDynUtils _fixed_robot;
    iDynUtils _normal_robot;
    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string _path_to_cfg;

    testCoMTask()
        : _robot("coman",
                 std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                 std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
          _fixed_robot("coman",
                       std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                       std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
          _normal_robot("coman",
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

    virtual ~testCoMTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testCoMTask, testCoMTask_)
{
    // setting initial position with bent legs
    Eigen::VectorXd q_whole(_robot.iDynTree_model.getNrOfDOFs());
    q_whole = Eigen::VectorXd::Constant(q_whole.size(), 1E-4);
    q_whole[_robot.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_robot.iDynTree_model.getDOFIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _robot.switchAnchorAndFloatingBase("l_sole");
    _robot.updateiDynTreeModel(q_whole, true);

    _fixed_robot.switchAnchorAndFloatingBase("l_sole");
    _fixed_robot.updateiDynTreeModel(q_whole);

    _normal_robot.updateiDynTreeModel(q_whole);

    std::cout << "_robot.getCoM() is: " << _robot.getCOM() << std::endl;
    std::cout << "_robot.getCoM(\"l_sole\") is: " << _robot.iDynTree_model.getCOM(
                     _robot.iDynTree_model.getLinkIndex("l_sole")).toString() << std::endl;

    std::cout << "_fixed_robot.getCoM() is: " << _fixed_robot.getCOM() << std::endl;
    std::cout << "_fixed_robot.getCoM(_\"l_sole\") is: " << _fixed_robot.iDynTree_model.getCOM(
                     _robot.iDynTree_model.getLinkIndex("l_sole")).toString() << std::endl;

    std::cout << "computing _normal_robot with world in waist.." << std::endl;
    std::cout << "_normal_robot.getCoM() is: " << _normal_robot.getCOM() << std::endl;
    std::cout << "_normal_robot.getCoM(_\"l_sole\") is: " << _normal_robot.iDynTree_model.getCOM(
                     _robot.iDynTree_model.getLinkIndex("l_sole")).toString() << std::endl;

    _normal_robot.updateiDynTreeModel(q_whole, true);

    std::cout << "computing _normal_robot with proper world" << std::endl;
    std::cout << "_normal_robot.getCoM() is: " << _normal_robot.getCOM() << std::endl;
    std::cout << "_normal_robot.getCoM(_\"l_sole\") is: " << _normal_robot.iDynTree_model.getCOM(
                     _robot.iDynTree_model.getLinkIndex("l_sole")).toString() << std::endl;


    OpenSoT::tasks::velocity::CoM CoM(q_whole, *(_model_ptr.get()));

    EXPECT_TRUE(CoM.getb() == Eigen::VectorXd::Zero(3)) << "b = " << CoM.getb();

    // setting x_ref with a delta offset along the z axis (+2cm)
    Eigen::VectorXd delta_x(3); delta_x.setZero(3);
                    delta_x(2) = 0.02;
    Eigen::VectorXd x = _robot.getCOM();
    Eigen::VectorXd x_ref = x + delta_x;

    Eigen::MatrixXd J;
    // hack! we need to compute world position in a smarter way....
    _fixed_robot.updateiDynTreeModel(q_whole,true);
    _fixed_robot.getCOMJacobian(J);
    J = J.block(0,6,3,q_whole.size());
    EXPECT_TRUE(CoM.getA() == J);
    EXPECT_EQ(CoM.getA().rows(), 3);
    EXPECT_EQ(CoM.getb().size(), 3);

    EXPECT_TRUE(conversion_utils_YARP::toYARP(CoM.getWeight()) == yarp::sig::Matrix(3,3).eye());

    EXPECT_TRUE(CoM.getConstraints().size() == 0);

    double K = 0.7;
    CoM.setLambda(K);
    EXPECT_DOUBLE_EQ(CoM.getLambda(), K);

    CoM.setReference(x);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],0,1E-12) << "b[i] = " << CoM.getb()[i];

    CoM.setReference(x_ref);
    Eigen::VectorXd positionError = x_ref - x;
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],CoM.getLambda()*positionError[i],1E-12) << "b[i] = " << CoM.getb()[i];

    _robot.updateiDynTreeModel(q_whole, true);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],CoM.getLambda()*positionError[i],1E-12) << "b[i] = " << CoM.getb()[i];

    Eigen::VectorXd x_now;
    SVDPseudoInverse<Eigen::MatrixXd> _pinv(CoM.getA(), 1E-6);
    for(unsigned int i = 0; i < 100; ++i)
    {
        _robot.updateiDynTreeModel(q_whole, true);

        CoM.update(q_whole);

        Eigen::MatrixXd Apinv;
        _pinv.compute(CoM.getA(), Apinv);
        q_whole += Apinv*CoM.getb();

        _robot.updateiDynTreeModel(q_whole, true);
        x_now = _robot.getCOM();
        std::cout << "Current error after iteration " << i << " is " << x_ref(2) - x_now(2) << std::endl;
    }


    EXPECT_LT( (x_ref - x_now).maxCoeff(), 1E-3 ) << "x_ref:" << x_ref << std::endl
                                                << "x_now:" << x_now << std::endl;
    EXPECT_LT( abs((x_ref - x_now).minCoeff()), 1E-3 );

    // checking for the position
    for(unsigned int i = 0; i < 3; ++i) {
        EXPECT_NEAR(x_ref(i),x_now(i),1E-4);
    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
