#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/variables/Torque.h>
#include <gtest/gtest.h>


namespace {

class testAffineHelper: public ::testing::Test
{
protected:

    testAffineHelper()
    {

    }

    virtual ~testAffineHelper() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testAffineHelper, checkBasics)
{
    std::vector<std::pair<std::string, int>> name_size_pairs;
    
    name_size_pairs.emplace_back("var1", 10);
    name_size_pairs.emplace_back("var2", 9);
    name_size_pairs.emplace_back("var3", 1);
    name_size_pairs.emplace_back("var4", 5);
    
    OpenSoT::OptvarHelper opt_helper(name_size_pairs);
    
    EXPECT_EQ( opt_helper.getSize(), 10+9+1+5 );
    
    
    OpenSoT::AffineHelper var1 = opt_helper.getVariable("var1");
    OpenSoT::AffineHelper var2 = opt_helper.getVariable("var2");
    OpenSoT::AffineHelper var3 = opt_helper.getVariable("var3");
    OpenSoT::AffineHelper var4 = opt_helper.getVariable("var4");
    
    EXPECT_EQ( var1.getInputSize(), opt_helper.getSize() );
    EXPECT_EQ( var1.getOutputSize(), 10 );
    
    EXPECT_EQ( var2.getInputSize(), opt_helper.getSize() );
    EXPECT_EQ( var2.getOutputSize(), 9 );
    
    EXPECT_EQ( var3.getInputSize(), opt_helper.getSize() );
    EXPECT_EQ( var3.getOutputSize(), 1 );
    
    EXPECT_EQ( var4.getInputSize(), opt_helper.getSize() );
    EXPECT_EQ( var4.getOutputSize(), 5 );
    
    Eigen::VectorXd x(opt_helper.getSize());
    for(int i = 0; i < x.size(); i++){
        x(i) = i;
    }
    
    std::cout << "x: " << x.transpose() << std::endl;
    
    Eigen::VectorXd val;
    
    var1.getValue(x, val);
    std::cout << "var1: " << val.transpose() << std::endl;
    EXPECT_EQ( ( val - x.segment(0, var1.getOutputSize()) ).norm(), 0.000 );
    EXPECT_EQ( (val - var1.getM()*x - var1.getq()).norm(), 0 );
    
    var2.getValue(x, val);
    std::cout << "var2: " << val.transpose() << std::endl;
    EXPECT_EQ( ( val - x.segment(10, var2.getOutputSize()) ).norm(), 0.000 );
    EXPECT_EQ( (val - var2.getM()*x - var2.getq()).norm(), 0 );
    
    var3.getValue(x, val);
    std::cout << "var3: " << val.transpose() << std::endl;
    EXPECT_EQ( ( val - x.segment(19, var3.getOutputSize()) ).norm(), 0.000 );
    EXPECT_EQ( (val - var3.getM()*x - var3.getq()).norm(), 0 );
    
    var4.getValue(x, val);
    std::cout << "var4: " << val.transpose() << std::endl;
    EXPECT_EQ( ( val - x.segment(20, var4.getOutputSize()) ).norm(), 0.000 );
    EXPECT_EQ( (val - var4.getM()*x - var4.getq()).norm(), 0 );
    
    
}


TEST_F( testAffineHelper, checkOperators ){
    
    int n = 20, m = 15;
    
    Eigen::MatrixXd M1;
    Eigen::VectorXd q1;
    
    M1.setRandom(m,n);
    q1.setRandom(m);
    
    Eigen::MatrixXd M2;
    Eigen::VectorXd q2;
    
    M2.setRandom(m,n);
    q2.setRandom(m);
    
    OpenSoT::AffineHelper aff1(M1, q1);
    OpenSoT::AffineHelper aff2(M2, q2);
    
    Eigen::MatrixXd A;
    A.setRandom(10, aff1.getOutputSize());
    
    OpenSoT::AffineHelper aff1prod = A*aff1;
    
    EXPECT_NEAR( (A*aff1.getM() - aff1prod.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*aff1.getq() - aff1prod.getq()).norm(), 0, 0.000001 );
    
    
//     auto aff_sum = aff1 + aff2;
//     
//     std::cout << aff1 << "\n\n\n" << aff2 << "\n\n\n" << aff_sum << std::endl;
//     
//     EXPECT_NEAR( (aff1.getM() + aff2.getM() - aff_sum.getM()).norm(), 0, 0.0001 );
//     EXPECT_NEAR( (aff1.getq() + aff2.getq() - aff_sum.getq()).norm(), 0, 0.0001 );
}

}


TEST_F( testAffineHelper, checkAffineBase ){
    
    int n = 20, m = 15;
    
    Eigen::MatrixXd M1;
    Eigen::VectorXd q1;
    
    M1.setRandom(m,n);
    q1.setRandom(m);
    
    OpenSoT::AffineHelper aff1(M1, q1);
    
    Eigen::MatrixXd M2;
    Eigen::VectorXd q2;
    
    M2.setRandom(m,n);
    q2.setRandom(m);
    
    OpenSoT::AffineHelper aff2(M2, q2);
    
    OpenSoT::AffineHelper aff_sum = aff1 + aff2;
    
    EXPECT_NEAR( (aff1.getM() + aff2.getM() - aff_sum.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (aff1.getq() + aff2.getq() - aff_sum.getq()).norm(), 0, 0.000001 );
    
    Eigen::MatrixXd A;
    A.setRandom(5, 15);
    
    OpenSoT::AffineHelper aff_prod = A * aff1;
    
    EXPECT_NEAR( (A*aff1.getM() - aff_prod.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*aff1.getq() - aff_prod.getq()).norm(), 0, 0.000001 );
    
    aff_prod = A*(aff1+aff2);
    
    EXPECT_NEAR( (A*aff_sum.getM() - aff_prod.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*aff_sum.getq() - aff_prod.getq()).norm(), 0, 0.000001 );
    
    Eigen::VectorXd b;
    b.setRandom(aff1.getOutputSize());
    
    OpenSoT::AffineHelper aff_vec_sum = aff1 + b;
    
    EXPECT_NEAR( (aff1.getM() - aff_vec_sum.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (aff1.getq() + b - aff_vec_sum.getq()).norm(), 0, 0.000001 );
    
    b.setRandom(A.rows());
    OpenSoT::AffineHelper aff_gemm = A*aff1 + b;
    
    EXPECT_NEAR( (A*aff1.getM() - aff_gemm.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*aff1.getq() + b - aff_gemm.getq()).norm(), 0, 0.000001 );
    
    A.setRandom(aff1.getOutputSize(), aff1.getOutputSize());
    aff_prod = A*A*aff1;
    EXPECT_NEAR( (A*A*aff1.getM() - aff_prod.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*A*aff1.getq() - aff_prod.getq()).norm(), 0, 0.000001 );
    
    A.setRandom(4, 10);
    Eigen::MatrixXd B;
    B.setRandom(10, aff1.getOutputSize());
    b.setRandom(A.rows());
    
    aff_gemm = A*B*aff1 + b;
    
    EXPECT_NEAR( (A*B*aff1.getM() - aff_gemm.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*B*aff1.getq() + b - aff_gemm.getq()).norm(), 0, 0.000001 );
    
    M2.setRandom(A.rows(), aff1.getInputSize());
    q2.setRandom(A.rows());
    aff2 = OpenSoT::AffineHelper(M2, q2);
    aff_gemm = A*B*aff1 + aff2;
    
    EXPECT_NEAR( (A*B*aff1.getM() + aff2.getM() - aff_gemm.getM()).norm(), 0, 0.000001 );
    EXPECT_NEAR( (A*B*aff1.getq() + aff2.getq() - aff_gemm.getq()).norm(), 0, 0.000001 );
    
    b.setRandom(aff1.getOutputSize());
    
    aff2 = aff1;
    aff1 = aff1 + b;
    
    EXPECT_NEAR( ( aff2.getq() + b - aff1.getq() ).norm(), 0, 0.00001 );
    
}


TEST_F( testAffineHelper, checkOperatorPile )
{
    Eigen::MatrixXd M1, M2;
    Eigen::VectorXd q1, q2;
    
    M1.setRandom(20, 50);
    M2.setRandom(15, M1.cols());
    
    q1.setRandom(M1.rows());
    q2.setRandom(M2.rows());
    
    OpenSoT::AffineHelper aff1(M1, q1);
    OpenSoT::AffineHelper aff2(M2, q2);
    
    auto aff3 = aff1 / aff2;
    
    EXPECT_EQ( (aff3.getM().block(0,0,aff1.getOutputSize(),aff1.getInputSize()) - aff1.getM()).norm(), 0 );
    EXPECT_EQ( (aff3.getM().bottomRows(aff2.getOutputSize()) - aff2.getM()).norm(), 0 );
    
}

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

TEST_F( testAffineHelper, checkTorque )
{
    auto model = XBot::ModelInterface::getModel(_path_to_cfg);
    
    model->setJointPosition(Eigen::VectorXd::Random(model->getJointNum()));
    model->setJointVelocity(Eigen::VectorXd::Random(model->getJointNum()));
    model->setJointAcceleration(Eigen::VectorXd::Random(model->getJointNum()));
    model->update();
    
    OpenSoT::OptvarHelper::VariableVector vars = {
                                                    {"qddot", model->getJointNum()}
                                                 };
    
    OpenSoT::OptvarHelper opt(vars);
    
    auto qddot = opt.getVariable("qddot");
    
    OpenSoT::variables::Torque tau(model, qddot);
    
    tau.update();
    
    std::cout << tau << std::endl;
    
    Eigen::VectorXd optvar_value, tau_value;
    model->getJointAcceleration(optvar_value);
    
    tau.getValue(optvar_value, tau_value);
    
    Eigen::VectorXd tau_id;
    
    model->computeInverseDynamics(tau_id);
    
    std::cout << "tau val " << tau_value.transpose() << std::endl;
    std::cout << "tau id " << tau_id.transpose() << std::endl;
    
    EXPECT_NEAR( (tau_id.tail(model->getActuatedJointNum()) - tau_value).norm(), 0, 0.0001 );
    
    Eigen::MatrixXd A(6, tau.getOutputSize());
    Eigen::VectorXd b(6);
    
    OpenSoT::AffineHelper task = A*tau + b;
    
    Eigen::MatrixXd _A = task.getM();
    Eigen::VectorXd _b = task.getq();
    
    
    
    
    
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
