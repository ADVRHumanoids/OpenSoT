#include <Eigen/Dense>
#include <yarp/sig/Matrix.h>
//#define __USE_SINGLE_PRECISION__
#include <qpOASES/QProblemB.hpp>
#include <qpOASES/SQProblem.hpp>
#include <yarp/os/Time.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include <yarp/math/RandnVector.h>
#include <yarp/math/RandVector.h>



#include <gtest/gtest.h>

namespace{
class testBasicAlgebra: public ::testing::Test{
public:
    testBasicAlgebra()
    {

    }

    ~testBasicAlgebra(){}
    virtual void SetUp(){}
    virtual void TearDown(){}

    yarp::sig::Matrix Ay;
    Eigen::MatrixXd Aed;
    Eigen::MatrixXf Aef;

    yarp::sig::Matrix Wy;
    Eigen::MatrixXd Wed;
    Eigen::MatrixXf Wef;

    yarp::sig::Vector by;
    Eigen::VectorXd bed;
    Eigen::VectorXf bef;



};

unsigned int iter = 10000;
unsigned int rows = 12;
unsigned int cols = 30;

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

TEST_F(testBasicAlgebra, testPinvVSQP)
{
    Eigen::MatrixXd A(rows, cols);
    Eigen::MatrixXd H(cols, cols);
    Eigen::VectorXd b(rows);
    Eigen::VectorXd g(rows);

    Eigen::VectorXd b_max(cols); b_max = b_max.setOnes(cols);
    Eigen::VectorXd b_min(cols); b_min = -b_max.setOnes(cols);

    Eigen::VectorXd t(iter);
    Eigen::VectorXd t2(iter);

    Eigen::VectorXd sol1(cols);
    Eigen::VectorXd sol2(cols);
    for(unsigned int i = 0; i < iter; ++i)
    {
        A.setRandom(rows, cols);
        b.setRandom(rows);

        double tic = yarp::os::Time::now()*1e6;
        sol1 = pseudoInverse(A)*b;
        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;

        H = A.transpose()*A;
        g = -1.0 * A.transpose()*b;

        qpOASES::SQProblem solver(cols, 0, qpOASES::HST_SEMIDEF);
        qpOASES::Options opt;
        opt.setToMPC();
        solver.setOptions(opt);
        solver.setPrintLevel(qpOASES::PL_NONE);
        int nrws = 64;


        tic = yarp::os::Time::now()*1e6;
        solver.init(H.data(), g.data(),NULL, b_min.data(), b_max.data(),NULL, NULL, nrws);
        solver.getPrimalSolution(sol2.data());
        toc = yarp::os::Time::now()*1e6;

        t2[i] = toc-tic;


        Eigen::VectorXd s1 = A*sol1;
        Eigen::VectorXd s2 = A*sol2;
        for(unsigned int i = 0; i < s1.size(); ++i)
        {
            EXPECT_NEAR(s1[i], b[i], 1e-5);
            EXPECT_NEAR(s2[i], b[i], 1e-5);
        }



        //std::cout<<"sol 1:"<<sol1<<std::endl;
        //std::cout<<"sol 2:"<<sol2<<std::endl;
    }

    std::cout<<"PSEUDOINVERSE----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
    std::cout<<"QPOASES----> Mean time for "<<iter<<" iterations: "<<t2.sum()/iter<<" us"<<std::endl;
}


TEST_F(testBasicAlgebra, testQPPVM)
{
    Eigen::VectorXd t(iter);
    Eigen::VectorXd t2(iter);

    Eigen::MatrixXd H;
    Eigen::VectorXd g;

    Eigen::MatrixXd H2;
    Eigen::VectorXd g2;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Eigen::MatrixXd J = Aed.setRandom(rows, cols);
        Eigen::VectorXd f = bed.setRandom(rows);

        Eigen::MatrixXd A;
        Eigen::VectorXd b;

        double tic = yarp::os::Time::now()*1e6;
        A = J;
        b = J*J.transpose()*f;
        H = A.transpose()*A;
        g = A.transpose()*b;
        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;

        tic = yarp::os::Time::now()*1e6;
        J = (J.transpose()).eval();
        A = pseudoInverse(J);
        b = f;
        H2 = A.transpose()*A;
        g2 = A.transpose()*b;
        toc = yarp::os::Time::now()*1e6;

        t2[i] = toc-tic;
    }


    std::cout<<"QPPVM EIGEN DOUBLE----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
    std::cout<<"QPPVM EIGEN DOUBLE----> Mean time using PseudoInverse for "<<iter<<" iterations: "<<t2.sum()/iter<<" us"<<std::endl;
    EXPECT_LE(t.sum()/iter, t2.sum()/iter);
}

TEST_F(testBasicAlgebra, testMulEigenDouble)
{
    Eigen::VectorXd t(iter);

    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Aed.setRandom(rows, cols);
        Wed.setRandom(rows,rows);
        Wed = (Wed + Wed.transpose()).eval();
        bed.setRandom(rows);

        double tic = yarp::os::Time::now()*1e6;
        H = Aed.transpose()*Wed*Aed;
        g = Aed.transpose()*Wed*bed;
        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;
    }


    std::cout<<"EIGEN DOUBLE----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
}

TEST_F(testBasicAlgebra, testMulEigenFloat)
{
    Eigen::VectorXf t(iter);

    Eigen::MatrixXf H;
    Eigen::VectorXf g;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Aef.setRandom(rows, cols);
        Wef.setRandom(rows,rows);
        Wef = (Wef + Wef.transpose()).eval();
        bef.setRandom(rows);

        double tic = yarp::os::Time::now()*1e6;
        H = Aef.transpose()*Wef*Aef;
        g = Aef.transpose()*Wef*bef;
        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;
    }


    std::cout<<"EIGEN FLOAT-----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
}

TEST_F(testBasicAlgebra, testMulYarp)
{
    using namespace yarp::math;


    Eigen::VectorXf t(iter);

    yarp::sig::Matrix H;
    yarp::sig::Vector g;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Ay = Rand::matrix(rows, cols);
        Wy = Rand::matrix(rows, rows);
        Wy = Wy + Wy.transposed();
        by = Rand::vector(rows);

        double tic = yarp::os::Time::now()*1e6;
        H = Ay.transposed()*Wy*Ay;
        g = Ay.transposed()*Wy*by;
        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;
    }


    std::cout<<"YARP DOUBLE-----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
}



TEST_F(testBasicAlgebra, testqpOASES)
{
#ifndef __USE_SINGLE_PRECISION__
    Eigen::VectorXd t(iter);

    Eigen::MatrixXd H;
    Eigen::VectorXd g;

    Eigen::VectorXd b_max(cols); b_max.setOnes(cols);
    Eigen::VectorXd b_min(cols); b_min = -b_max.setOnes(cols);


    int nwsr = 32;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Aed.setRandom(rows, cols);
        Wed.setRandom(rows,rows);
        Wed = (Wed + Wed.transpose()).eval();
        bed.setRandom(rows);

        double tic = yarp::os::Time::now()*1e6;
        H = Aed.transpose()*Wed*Aed;
        g = Aed.transpose()*Wed*bed;


        qpOASES::QProblemB solver(cols);
        solver.setPrintLevel(qpOASES::PL_NONE);

        solver.init(H.data(), g.data(), b_min.data(), b_max.data(), nwsr);

        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;
    }


    std::cout<<"QPOASES EIGEN DOUBLE----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
#else
    Eigen::VectorXf t(iter);

    Eigen::MatrixXf H;
    Eigen::VectorXf g;

    Eigen::VectorXf b_max(cols); b_max.setOnes(cols);
    Eigen::VectorXf b_min(cols); b_min = -b_max.setOnes(cols);

    int nwsr = 32;
    for(unsigned int i = 0; i < iter; ++i)
    {
        Aef.setRandom(rows, cols);
        Wef.setRandom(rows,rows);
        Wef = (Wef + Wef.transpose()).eval();
        bef.setRandom(rows);

        double tic = yarp::os::Time::now()*1e6;
        H = Aef.transpose()*Wef*Aef;
        g = Aef.transpose()*Wef*bef;

        qpOASES::QProblemB solver(cols);
        solver.setPrintLevel(qpOASES::PL_NONE);

        solver.init(H.data(), g.data(), b_min.data(), b_max.data(), nwsr);

        double toc = yarp::os::Time::now()*1e6;

        t[i] = toc-tic;
    }


    std::cout<<"QPOASES EIGEN FLOAT-----> Mean time for "<<iter<<" iterations: "<<t.sum()/iter<<" us"<<std::endl;
#endif
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
