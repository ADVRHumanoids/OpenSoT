#include <OpenSoT/utils/Piler.h>
#include <gtest/gtest.h>

namespace{

class testPiler: public ::testing::Test
{
protected:

    testPiler()
    {

    }

    virtual ~testPiler() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

inline void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    A.conservativeResize(A.rows()+B.rows(), A.cols());
    A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
}

TEST_F(testPiler, checkPiler)
{
    int ncols = 50;
    OpenSoT::utils::MatrixPiler piler(50);

    Eigen::MatrixXd A;
    A.setRandom(23, ncols);

    piler.pile(A);

    EXPECT_TRUE( ( (A - piler.generate_and_get()).array() == 0).all() );


    piler.reset();

    int N = 10;
    Eigen::MatrixXd Apiled(0, ncols);

    for(int i = 0; i < N; i++)
    {
        int nrows = 2*i + 1;
        A.setRandom(nrows, ncols);

        piler.pile(A);
        pile(Apiled, A);



        EXPECT_TRUE( ( (Apiled - piler.generate_and_get()).array() == 0).all() );

    }

    EXPECT_TRUE( ( (Apiled - piler.generate_and_get()).array() == 0).all() );


    piler.set(A);

    EXPECT_TRUE( ( (A - piler.generate_and_get()).array() == 0).all() );

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

