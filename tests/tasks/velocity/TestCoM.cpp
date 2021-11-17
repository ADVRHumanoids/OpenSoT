#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <XBotInterface/ModelInterface.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

namespace {

class testCoMTask: public ::testing::Test
{
public:

protected:

    testCoMTask()
    {
        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

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

    XBot::ModelInterface::Ptr _model_ptr;

};

TEST_F(testCoMTask, testCoMTask_)
{
    // setting initial position with bent legs
    Eigen::VectorXd q_whole(_model_ptr->getJointNum());
    q_whole = Eigen::VectorXd::Constant(q_whole.size(), 1E-4);
    q_whole[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;


    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    OpenSoT::tasks::velocity::CoM CoM(q_whole, *(_model_ptr.get()));

    EXPECT_TRUE(CoM.getb() == Eigen::VectorXd::Zero(3)) << "b = " << CoM.getb();

    // setting x_ref with a delta offset along the z axis (+2cm)
    Eigen::Vector3d delta_x; delta_x.setZero();
                    delta_x(2) = 0.02;
    Eigen::Vector3d x;
    _model_ptr->getCOM(x);
    Eigen::Vector3d x_ref = x + delta_x;

    Eigen::MatrixXd J;
    _model_ptr->getCOMJacobian(J);

    // hack! we need to compute world position in a smarter way....
    EXPECT_TRUE(CoM.getA() == J);
    EXPECT_EQ(CoM.getA().rows(), 3);
    EXPECT_EQ(CoM.getb().size(), 3);

    EXPECT_TRUE(CoM.getWeight() == Eigen::MatrixXd::Identity(3,3));

    EXPECT_TRUE(CoM.getConstraints().size() == 0);

    double K = 0.7;
    CoM.setLambda(K);
    EXPECT_DOUBLE_EQ(CoM.getLambda(), K);

    CoM.setReference(x);
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],0,1E-12) << "b[i] = " << CoM.getb()[i];

    CoM.setReference(x_ref);
    CoM.update(q_whole);
    Eigen::VectorXd positionError = x_ref - x;
    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_NEAR(CoM.getb()[i],CoM.getLambda()*positionError[i],1E-12) << "b[i] = " << CoM.getb()[i];

    Eigen::Vector3d x_now;
    SVDPseudoInverse<Eigen::MatrixXd> _pinv(CoM.getA(), 1E-6);
    for(unsigned int i = 0; i < 100; ++i)
    {
        _model_ptr->setJointPosition(q_whole);
        _model_ptr->update();

        CoM.update(q_whole);

        Eigen::MatrixXd Apinv;
        _pinv.compute(CoM.getA(), Apinv);
        q_whole += Apinv*CoM.getb();

        _model_ptr->getCOM(x_now);
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

TEST_F(testCoMTask, testReset)
{
    Eigen::VectorXd q_whole(_model_ptr->getJointNum());
    q_whole = Eigen::VectorXd::Constant(q_whole.size(), 1E-4);
    q_whole[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q_whole[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    OpenSoT::tasks::velocity::CoM CoM(q_whole, *(_model_ptr.get()));

    std::cout<<"INITIALIZATION: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    Eigen::Vector3d actual_pose = CoM.getActualPosition();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    Eigen::Vector3d reference_pose = CoM.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<CoM.getb()<<std::endl;

    for(unsigned int j = 0; j < 3; ++j)
        EXPECT_EQ(actual_pose[j], reference_pose[j]);

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_EQ(CoM.getb()[i], 0);

    std::cout<<"CHANGING q: ACTUAL AND REFERENCE DIFFERENT, b IS NOT 0"<<std::endl;
    q_whole.setRandom(q_whole.size());
    _model_ptr->setJointPosition(q_whole);
    _model_ptr->update();

    CoM.update(q_whole);

    actual_pose = CoM.getActualPosition();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    reference_pose = CoM.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<CoM.getb()<<std::endl;

    std::cout<<"RESET: ACTUAL AND REFERENCE EQUAL, b IS 0"<<std::endl;
    EXPECT_TRUE(CoM.reset());
    actual_pose = CoM.getActualPosition();
    std::cout<<"actual_pose: \n"<<actual_pose<<std::endl;
    reference_pose = CoM.getReference();
    std::cout<<"reference_pose: \n"<<reference_pose<<std::endl;
    std::cout<<"b: \n"<<CoM.getb()<<std::endl;

    for(unsigned int j = 0; j < 3; ++j)
        EXPECT_EQ(actual_pose[j], reference_pose[j]);

    for(unsigned int i = 0; i < 3; ++i)
        EXPECT_EQ(CoM.getb()[i], 0);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
