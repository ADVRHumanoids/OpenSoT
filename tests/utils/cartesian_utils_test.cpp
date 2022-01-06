#include <gtest/gtest.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <random>

namespace {

class testQuaternion: public ::testing::Test, public quaternion
{
protected:
    testQuaternion()
    {

    }

    virtual ~testQuaternion() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

class testCartesianUtils: public ::testing::Test
{

    class sin: public CostFunction
    {
        double compute(const Eigen::VectorXd &x)
        {
            return std::sin(x[0]);
        }
    };

protected:
    sin sin_function;

    testCartesianUtils()
    {

    }

    virtual ~testCartesianUtils() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testCartesianUtils, testPseudoInverse1)
{
    Eigen::MatrixXd A(32,32);
    A.setZero();
    Eigen::MatrixXd Ainv(32,32);
    Ainv.setZero();
    Eigen::MatrixXd Apinv(32,32);
    Apinv.setZero();

    SVDPseudoInverse<Eigen::MatrixXd> pinv(A);


    for(unsigned int i = 0; i < 1000; ++i)
    {
        srand((unsigned int) time(0));
        A.setRandom();

        Ainv = A.inverse();
        pinv.compute(A, Apinv);

        for(unsigned int j = 0; j < A.rows(); ++j)
        {
            for(unsigned int k = 0; k < A.cols(); ++k)
                EXPECT_NEAR(Ainv(j,k), Apinv(j,k), 1e-8);
        }
    }

    srand((unsigned int) time(0));
    A.setRandom();
    LDLTInverse<Eigen::MatrixXd> LDLTinv(A);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        srand((unsigned int) time(0));
        A.setRandom();
        A = (A*A.transpose()).eval();

        Ainv = A.inverse();
        LDLTinv.compute(A, Apinv);

        for(unsigned int j = 0; j < A.rows(); ++j)
        {
            for(unsigned int k = 0; k < A.cols(); ++k)
                EXPECT_NEAR(Ainv(j,k), Apinv(j,k), 1e-6);
        }
    }
}

TEST_F(testQuaternion, testQuaternionError)
{
    EXPECT_DOUBLE_EQ(this->x, 0.0);
    EXPECT_DOUBLE_EQ(this->y, 0.0);
    EXPECT_DOUBLE_EQ(this->z, 0.0);
    EXPECT_DOUBLE_EQ(this->w, 1.0);

    KDL::Rotation rot_desired;
    rot_desired.Identity();
    rot_desired.DoRotZ(M_PI_2);
    double x, y, z, w;
    rot_desired.GetQuaternion(x, y, z, w);
    quaternion q2(x, y, z, w);
    EXPECT_DOUBLE_EQ(q2.x, x);
    EXPECT_DOUBLE_EQ(q2.y, y);
    EXPECT_DOUBLE_EQ(q2.z, z);
    EXPECT_DOUBLE_EQ(q2.w, w);

    double dot_product = quaternion::dot(quaternion(this->x, this->y, this->z, this->w), q2);
    EXPECT_DOUBLE_EQ(dot_product, w*this->w);

    std::uniform_real_distribution<double> unif;
    std::mt19937 re;

    double a = unif(re);
    EXPECT_DOUBLE_EQ(a*q2.x, a*x);
    EXPECT_DOUBLE_EQ(a*q2.y, a*y);
    EXPECT_DOUBLE_EQ(a*q2.z, a*z);
    EXPECT_DOUBLE_EQ(a*q2.w, a*w);

    KDL::Rotation skew_q2 = q2.skew();
    EXPECT_DOUBLE_EQ(skew_q2(0,0), 0.0);   EXPECT_DOUBLE_EQ(skew_q2(0,1), -q2.z); EXPECT_DOUBLE_EQ(skew_q2(0,2), q2.y);
    EXPECT_DOUBLE_EQ(skew_q2(1,0), q2.z);  EXPECT_DOUBLE_EQ(skew_q2(1,1), 0.0);   EXPECT_DOUBLE_EQ(skew_q2(1,2), -q2.x);
    EXPECT_DOUBLE_EQ(skew_q2(2,0), -q2.y); EXPECT_DOUBLE_EQ(skew_q2(2,1), q2.x);  EXPECT_DOUBLE_EQ(skew_q2(2,2), 0.0);

    quaternion q1(q2.x, q2.y, q2.z, q2.w);
    KDL::Vector quaternion_error = quaternion::error(q1, q2);
    EXPECT_DOUBLE_EQ(quaternion_error[0], 0.0);
    EXPECT_DOUBLE_EQ(quaternion_error[1], 0.0);
    EXPECT_DOUBLE_EQ(quaternion_error[2], 0.0);
}

TEST_F(testCartesianUtils, testComputeCartesianError)
{
    Eigen::Vector3d position_error;
    position_error.setZero();
    Eigen::Vector3d orientation_error;
    orientation_error.setZero();

    KDL::Rotation rot;
    rot.Identity();
    rot.DoRotZ(M_PI);

    double x = 1.0;
    double y = -1.0;
    double z = -2.0;

    double qx, qy, qz, qw;
    rot.GetQuaternion(qx, qy, qz, qw);
    Eigen::MatrixXd Td(4,4);
    Td.setZero();
    KDL::Frame tmp(KDL::Rotation::Quaternion(qx, qy, qz, qw), KDL::Vector(x, y, z));
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            Td(i,j) = tmp.M(i,j);
    }
    Td(0,3) = tmp.p.x();
    Td(1,3) = tmp.p.y();
    Td(2,3) = tmp.p.z();

    Eigen::MatrixXd T(4,4);
    T = Td;

    cartesian_utils::computeCartesianError(T, Td, position_error, orientation_error);

    for(unsigned int i = 0; i < position_error.rows(); ++i)
    {
        EXPECT_DOUBLE_EQ(position_error[i], 0.0);
        EXPECT_DOUBLE_EQ(orientation_error[i], 0.0);
    }
}

TEST_F(testCartesianUtils, testComputeGradient)
{
    int n_of_iterations = 100;
    double dT = 1.0 / ((double)n_of_iterations);

    for(unsigned int i = 0; i < n_of_iterations; ++i)
    {
        double df = cos(i * dT);

        Eigen::VectorXd x(1);
        x.setZero();
        x[0] = i*dT;
        Eigen::VectorXd df_numerical = cartesian_utils::computeGradient(x, this->sin_function, 1E-6);

        EXPECT_NEAR(df, df_numerical[0], 1E-6);
    }
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
