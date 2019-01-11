#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>
#include <boost/make_shared.hpp>
#include <XBotInterface/ModelInterface.h>


namespace{

class testCartesianAdmittanceTask: public ::testing::Test
{
protected:
    XBot::ModelInterface::Ptr _model_ptr;
    std::string _path_to_cfg;

    testCartesianAdmittanceTask()
    {
        std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
        std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";

        _path_to_cfg = robotology_root + relative_path;

        _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_model_ptr)
            std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
    }

    virtual ~testCartesianAdmittanceTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testCartesianAdmittanceTask, testComputeParameters)
{
    Eigen::VectorXd q(_model_ptr->getJointNum());
    q.setZero(q.size());

    XBot::ForceTorqueSensor::ConstPtr ft;
    ft.reset(new XBot::ForceTorqueSensor(_model_ptr->getUrdf().getLink("LSoftHand"), 0));

    OpenSoT::tasks::velocity::CartesianAdmittance::Ptr left_arm;
    left_arm.reset(new OpenSoT::tasks::velocity::CartesianAdmittance(
                       "left_arm", q, *_model_ptr, "world", ft));

    double dt = 0.001;
    left_arm->setFilterTimeStep(dt);

    double lambda = 0.001;
    left_arm->setLambda(lambda);

    Eigen::Vector6d K; K<<1000., 1000., 1000., 1000., 1000., 1000.;
    Eigen::Vector6d D; D<<10., 10., 10., 10., 10., 10.;

    left_arm->setStiffness(K);
    left_arm->setDamping(D);

    Eigen::Vector6d I; I.setOnes();

    Eigen::Vector6d _C = 1e-6*I; //computed from Matlab
    Eigen::Vector6d _M = 10.*I; //computed from Matlab
    Eigen::Vector6d _w = 100.*I; //computed from Matlab

    Eigen::Vector6d C;
    Eigen::Vector6d M;
    Eigen::Vector6d w;

    Eigen::Vector6d _K, _D;
    for(unsigned int i = 0; i < 6; ++i)
    {
        _K[i] = left_arm->getStiffness()(i,i);
        _D[i] = left_arm->getDamping()(i,i);
    }

    left_arm->computeParameters(_K, _D, left_arm->getLambda(), left_arm->getFilterTimeStep(), C, M, w);

    std::cout<<"DIRECT PROBLEM"<<std::endl;
    std::cout<<"_C: ["<<_C.transpose()<<"]"<<std::endl;
    std::cout<<"C: ["<<C.transpose()<<"]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"_M: ["<<_M.transpose()<<"]"<<std::endl;
    std::cout<<"M: ["<<M.transpose()<<"]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"_w: ["<<_w.transpose()<<"]"<<std::endl;
    std::cout<<"w: ["<<w.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(_C[i], C[i], 2e-8);
        EXPECT_NEAR(_M[i], M[i], 2e-8);
        EXPECT_NEAR(_w[i], w[i], 2e-8);
    }


    //Inverse Problem
    _C.segment(0,3) = 1e-6*I.segment(0,3);
    _C.segment(3,3) = 1e-7*I.segment(0,3);
    _w = 4.*M_PI*I;

    lambda = 0.01;
    left_arm->setLambda(lambda);
    dt = 0.002;
    left_arm->setFilterTimeStep(dt);

    K<<1e4, 1e4, 1e4, 1e5, 1e5, 1e5;
    D<<1e3*0.7958, 1e3*0.7958, 1e3*0.7958, 1e3*7.9577, 1e3*7.9577, 1e3*7.9577;

    left_arm->computeParameters(K, D, left_arm->getLambda(), left_arm->getFilterTimeStep(), C, M, w);

    std::cout<<"INVERSE PROBLEM"<<std::endl;
    std::cout<<"_C: ["<<_C.transpose()<<"]"<<std::endl;
    std::cout<<"C: ["<<C.transpose()<<"]"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"_w: ["<<_w.transpose()<<"]"<<std::endl;
    std::cout<<"w: ["<<w.transpose()<<"]"<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
    {
        EXPECT_NEAR(_C[i], C[i], 1e-3);
        EXPECT_NEAR(_w[i], w[i], 1e-3);
    }



}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
