#include <gtest/gtest.h>
#include <wb_sot/solvers/QPOasis.h>
#include <yarp/sig/all.h>

namespace {

class testSolverQPOases: public ::testing::Test
{
protected:

    testSolverQPOases():
        solver()
    {

    }

    virtual ~testSolverQPOases() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    wb_sot::solvers::QPOases<yarp::sig::Matrix, yarp::sig::Vector> solver;

};


TEST_F(testSolverQPOases, testSolverInitialization){
    EXPECT_EQ(0, solver.getNumberOfConstraints()) <<"Initial number of constraints has to be 0"<<std::endl;
    EXPECT_EQ(0, solver.getNumberOfBounds()) <<"Initial number of bounds has to be 0"<<std::endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
