#include <qpOASES/Matrices.hpp>
#include <gtest/gtest.h>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>

namespace
{
    class testqpOASESSparseMatrices: public ::testing::Test
    {
    protected:

        testqpOASESSparseMatrices()
        {

        }

        virtual ~testqpOASESSparseMatrices() {

        }

        virtual void SetUp() {

        }

        virtual void TearDown() {

        }

    };

    TEST_F(testqpOASESSparseMatrices, testSparseMatrices)
    {
        yarp::sig::Matrix sym_sparse_mat_yarp(4,4);
        sym_sparse_mat_yarp.eye();
        for(unsigned int i = 0; i < sym_sparse_mat_yarp.rows(); ++i)
            sym_sparse_mat_yarp(i,i) = double(i);
        sym_sparse_mat_yarp(0,3) = 5.0;
        sym_sparse_mat_yarp(3,0) = 5.0;

        qpOASES::SymSparseMat sym_sparse_mat_qpoases(sym_sparse_mat_yarp.rows(), sym_sparse_mat_yarp.cols(),
                                                     sym_sparse_mat_yarp.rows(), sym_sparse_mat_yarp.data());

        std::cout<<"SymSparseMat1:"<<std::endl;
        sym_sparse_mat_qpoases.print();

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::sparse_int_t H_jc[] = { 0,  4,  8, 12, 16, 20, 20, 20, 20, 20, 20,
                               24, 28, 32, 36, 40, 40, 40, 40, 40, 40,
                               44, 48, 52, 56, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
                               64, 68, 72, 76, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                               80, 80, 80, 80, 80, 80 };

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::real_t H_val[] = {10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1,
            1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 1,
            10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 1, 10, 1,
            1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10, 1, 1, 1, 10};

        /** Sparse Hessian matrix data for qrecipe example. */
        qpOASES::sparse_int_t H_ir[] = {
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38,
            0, 10, 20, 34, 1, 11, 21, 35, 2, 12, 22, 36, 3, 13, 23, 37, 4, 14, 24, 38};

        std::cout<<std::endl;
        qpOASES::SymSparseMat sym_sparse_mat_qpoases2(180, 180, H_ir, H_jc, H_val);
        std::cout<<"SymSparseMat2:"<<std::endl;
        sym_sparse_mat_qpoases2.print();


        std::cout<<std::endl;
        yarp::sig::Matrix sym_sparse_mat_yarp2(3,3);
        sym_sparse_mat_yarp2.eye();
        for(unsigned int i = 0; i < sym_sparse_mat_yarp2.rows(); ++i)
            sym_sparse_mat_yarp2(i,i) = double(i);
        sym_sparse_mat_yarp2(0,2) = 5.0;
        sym_sparse_mat_yarp2(2,0) = 5.0;

        qpOASES::SymSparseMat sym_sparse_mat_qpoases3(sym_sparse_mat_yarp2.rows(), sym_sparse_mat_yarp2.cols(),
                                                     3, sym_sparse_mat_yarp2.data());
        std::cout<<"SymSparseMat3:"<<std::endl;
        sym_sparse_mat_qpoases3.print();
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
